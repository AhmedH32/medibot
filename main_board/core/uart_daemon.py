"""
Deterministic 50 Hz UART Egress / 10 Hz Telemetry Ingress Daemon.
Maintains the 200ms Pico watchdog invariant, computes dead-reckoning odometry,
and enforces host-side stall and obstacle safety reflexes.
"""

from dataclasses import dataclass
import math
import threading
import time
import logging
from typing import Optional

from core.protocol import (
    CommandPacket,
    TelemetryPacket,
    TelemetryStreamParser,
    FLAG_MOTOR_ENABLE,
    FLAG_EMERGENCY_FALL,
)
from hal.serial_link import SerialLink

logger = logging.getLogger("MediBot.Core.UARTDaemon")


@dataclass
class OdometryPose:
    x_cm: float = 0.0
    y_cm: float = 0.0
    theta_deg: float = 0.0


class UARTDaemon:
    # Physical robot constants from Subsystem B specification
    WHEELBASE_CM = 24.0
    WHEEL_DIAMETER_CM = 6.5
    TICKS_PER_REV = 1320.0  # 11 PPR * 30:1 gear ratio * 4x quadrature
    CM_PER_TICK = (math.pi * WHEEL_DIAMETER_CM) / TICKS_PER_REV

    def __init__(
        self,
        port_url: str,
        baudrate: int = 115200,
        tx_hz: int = 50,
        sonar_stop_cm: int = 20,
    ):
        self.link = SerialLink(port_url=port_url, baudrate=baudrate, timeout_s=0.0)
        self.parser = TelemetryStreamParser()
        self.tx_period_s = 1.0 / float(tx_hz)
        self.sonar_stop_cm = sonar_stop_cm

        self._lock = threading.Lock()
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None

        # Command state (protected by _lock)
        self._target_cmd = CommandPacket(vx=0, wz=0, flags=FLAG_MOTOR_ENABLE)

        # Telemetry & Odometry state (protected by _lock)
        self._latest_telemetry: Optional[TelemetryPacket] = None
        self._last_rx_timestamp: float = 0.0
        self._pose = OdometryPose()

        # Host-side stall cooldown timer (prevents commanding into a carpet stall)
        self._stall_recovery_until: float = 0.0

        # Diagnostic counters
        self.tx_count = 0
        self.rx_count = 0

    def start(self) -> None:
        """Opens serial transport and starts the 50 Hz background worker."""
        if self._running:
            return
        self.link.open()
        self._running = True
        self._worker_thread = threading.Thread(target=self._run_loop, name="UARTDaemon50Hz", daemon=True)
        self._worker_thread.start()
        logger.info("UARTDaemon started at %.0f Hz.", 1.0 / self.tx_period_s)

    def stop(self) -> None:
        """Safely sends a zero-velocity standby packet and closes transport."""
        if not self._running:
            return
        self._running = False
        if self._worker_thread:
            self._worker_thread.join(timeout=0.5)
        try:
            # Put Pico motor driver into standby before disconnecting
            standby_pkt = CommandPacket(vx=0, wz=0, flags=0).pack()
            self.link.write(standby_pkt)
        except Exception:
            pass
        self.link.close()
        logger.info("UARTDaemon stopped.")

    def set_command(self, vx: int, wz: int, flags: int = FLAG_MOTOR_ENABLE) -> None:
        """Thread-safe update of desired robot velocity and state flags."""
        with self._lock:
            self._target_cmd = CommandPacket(vx=vx, wz=wz, flags=flags)

    def trigger_emergency_fall_lock(self) -> None:
        """Immediately engages Bit 1 (0x02) hard motor lock for fall emergencies."""
        with self._lock:
            self._target_cmd = CommandPacket(
                vx=0,
                wz=0,
                flags=FLAG_MOTOR_ENABLE | FLAG_EMERGENCY_FALL,
            )

    def get_telemetry(self) -> Optional[TelemetryPacket]:
        """Returns the most recent 8-byte telemetry snapshot from the Pico."""
        with self._lock:
            return self._latest_telemetry

    def get_pose(self) -> OdometryPose:
        """Returns a copy of the integrated (x, y, theta) odometry pose."""
        with self._lock:
            return OdometryPose(
                x_cm=self._pose.x_cm,
                y_cm=self._pose.y_cm,
                theta_deg=self._pose.theta_deg,
            )

    def _run_loop(self) -> None:
        next_tick = time.perf_counter()

        while self._running:
            now = time.perf_counter()

            # 1. Non-blocking Ingress: Drain and parse any incoming telemetry bytes
            raw_in = self.link.read(64)
            if raw_in:
                packets = self.parser.feed(raw_in)
                for pkt in packets:
                    self._process_telemetry(pkt, now)

            # 2. Apply Safety Reflex Overrides before transmitting
            with self._lock:
                cmd = self._target_cmd
                tel = self._latest_telemetry

                eff_vx = cmd.vx
                eff_wz = cmd.wz
                eff_flags = cmd.flags

                # Reflex A: Carpet/Wall Motor Stall Cooldown (2.0s hold)
                if now < self._stall_recovery_until:
                    eff_vx = 0
                    eff_wz = 0
                    eff_flags = 0  # Drop STBY to let TB6612FNG cool down

                # Reflex B: Forward Sonar Obstacle Override (< 20 cm)
                elif tel is not None and tel.sonar_dist_cm < self.sonar_stop_cm and eff_vx > 0:
                    eff_vx = 0  # Block forward motion, retain Wz so robot can turn away

                out_packet = CommandPacket(vx=eff_vx, wz=eff_wz, flags=eff_flags).pack()

            # 3. Deterministic 50 Hz Egress
            try:
                self.link.write(out_packet)
                self.tx_count += 1
            except Exception as e:
                logger.error("UART write error: %s", e)

            # 4. Drift-compensated sleep until next 20ms boundary
            next_tick += self.tx_period_s
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                # If host OS stalled severely, resync next_tick to avoid burst flooding
                next_tick = time.perf_counter()

    def _process_telemetry(self, pkt: TelemetryPacket, timestamp: float) -> None:
        """Updates telemetry state, integrates odometry, and checks stall flags."""
        with self._lock:
            self._latest_telemetry = pkt
            self._last_rx_timestamp = timestamp
            self.rx_count += 1

            # Trigger 2.0s motor cooldown if Pico flags a current stall (Byte 6 Bit 7)
            if pkt.is_stalled:
                logger.warning("MOTOR STALL DETECTED by Pico! Enforcing 2.0s cooldown.")
                self._stall_recovery_until = timestamp + 2.0

            # Differential drive dead-reckoning integration
            d_left = pkt.delta_ticks_left * self.CM_PER_TICK
            d_right = pkt.delta_ticks_right * self.CM_PER_TICK
            d_center = (d_left + d_right) / 2.0
            d_theta_rad = (d_right - d_left) / self.WHEELBASE_CM

            mid_theta_rad = math.radians(self._pose.theta_deg) + (d_theta_rad / 2.0)
            self._pose.x_cm += d_center * math.cos(mid_theta_rad)
            self._pose.y_cm += d_center * math.sin(mid_theta_rad)
            self._pose.theta_deg = (self._pose.theta_deg + math.degrees(d_theta_rad)) % 360.0