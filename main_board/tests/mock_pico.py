"""
RP2040 Digital Twin / Simulation Server.
Listens on TCP 127.0.0.1:5555, verifies 50 Hz commands, computes differential kinematics,
enforces the 200ms hardware watchdog, and streams 10 Hz telemetry.
"""

import socket
import time
import threading
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from core.protocol import (
    CommandPacket,
    TelemetryPacket,
    CMD_FRAME_SIZE,
    FLAG_MOTOR_ENABLE,
    FLAG_EMERGENCY_FALL,
)


class MockPicoServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 5555):
        self.host = host
        self.port = port
        self.running = False
        self._sock: socket.socket = None
        self._client_conn: socket.socket = None

        # State Variables
        self.last_packet_time = 0.0
        self.watchdog_triggered = True
        self.current_cmd = CommandPacket(vx=0, wz=0, flags=0)

        # Injected telemetry values for testing
        self.sim_pitch = 0
        self.sim_roll = 0
        self.sim_sonar_cm = 150
        self.sim_battery_pct = 95
        self.sim_is_stalled = False

        # Wheel ticks accumulation (30:1 gear, 11 PPR, 4x quad = 1320 ticks/rev, 6.5cm wheel)
        self.wheelbase_cm = 24.0
        self.cm_per_tick = (3.14159 * 6.5) / 1320.0

    def start(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.host, self.port))
        self._sock.listen(1)
        self.running = True

        print(f"[MockPico] Listening on {self.host}:{self.port} (Emulating RP2040 UART)...")
        self._accept_thread = threading.Thread(target=self._connection_worker, daemon=True)
        self._accept_thread.start()

    def _connection_worker(self) -> None:
        while self.running:
            try:
                conn, addr = self._sock.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.settimeout(0.005)
                print(f"[MockPico] Main Board connected from {addr}")
                self._client_conn = conn

                tx_thread = threading.Thread(target=self._telemetry_sender_loop, daemon=True)
                tx_thread.start()

                self._command_receiver_loop()
            except Exception as e:
                if self.running:
                    print(f"[MockPico] Connection reset: {e}")
                self._client_conn = None

    def _command_receiver_loop(self) -> None:
        buf = bytearray()
        while self.running and self._client_conn:
            try:
                chunk = self._client_conn.recv(64)
                if not chunk:
                    break
                buf.extend(chunk)

                while len(buf) >= CMD_FRAME_SIZE:
                    if buf[0] != 0xAA:
                        buf.pop(0)
                        continue

                    candidate = bytes(buf[:CMD_FRAME_SIZE])
                    cmd = CommandPacket.unpack(candidate)
                    if cmd:
                        self.last_packet_time = time.perf_counter()
                        self.watchdog_triggered = False
                        self.current_cmd = cmd
                        del buf[:CMD_FRAME_SIZE]
                    else:
                        buf.pop(0)
            except socket.timeout:
                pass
            except Exception:
                break

            # Watchdog Invariant Check (200ms)
            if (time.perf_counter() - self.last_packet_time) > 0.200 and not self.watchdog_triggered:
                self.watchdog_triggered = True
                print("[MockPico] ALARM: ERR_WATCHDOG_TIMEOUT (>200ms elapsed without command). MOTORS HALTED.")

    def _telemetry_sender_loop(self) -> None:
        # Drift-compensated 10 Hz (100ms) deadline timer
        next_deadline = time.perf_counter()
        while self.running and self._client_conn:
            if (
                self.watchdog_triggered
                or not (self.current_cmd.flags & FLAG_MOTOR_ENABLE)
                or (self.current_cmd.flags & FLAG_EMERGENCY_FALL)
            ):
                delta_l = 0
                delta_r = 0
            else:
                dt = 0.1
                v_x = self.current_cmd.vx
                w_rad = (self.current_cmd.wz * 3.14159) / 180.0
                v_left = v_x - (w_rad * self.wheelbase_cm / 2.0)
                v_right = v_x + (w_rad * self.wheelbase_cm / 2.0)

                delta_l = int(max(-127, min(127, (v_left * dt) / self.cm_per_tick)))
                delta_r = int(max(-127, min(127, (v_right * dt) / self.cm_per_tick)))

            tel = TelemetryPacket(
                pitch=self.sim_pitch,
                roll=self.sim_roll,
                delta_ticks_left=delta_l,
                delta_ticks_right=delta_r,
                sonar_dist_cm=self.sim_sonar_cm,
                battery_pct=self.sim_battery_pct,
                is_stalled=self.sim_is_stalled,
            )

            try:
                self._client_conn.sendall(tel.pack())
            except Exception:
                break

            next_deadline += 0.100
            sleep_duration = next_deadline - time.perf_counter()
            if sleep_duration > 0:
                time.sleep(sleep_duration)

    def stop(self) -> None:
        self.running = False
        if self._client_conn:
            self._client_conn.close()
        if self._sock:
            self._sock.close()


if __name__ == "__main__":
    server = MockPicoServer()
    server.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        server.stop()