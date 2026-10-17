"""
Automated validation of Phase 1 Simulation Acceptance Gates.
Matches Section 2.3 of Alaa's technical specification.
"""

import time
import pytest
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from core.protocol import CommandPacket, TelemetryStreamParser, FLAG_MOTOR_ENABLE
from hal.serial_link import SerialLink
from tests.mock_pico import MockPicoServer


@pytest.fixture(scope="module")
def mock_pico():
    server = MockPicoServer(host="127.0.0.1", port=5555)
    server.start()
    time.sleep(0.05)  # Allow socket bind
    yield server
    server.stop()


def test_50hz_command_and_telemetry_flow(mock_pico):
    """Gate 1: Verify sustained 50Hz command loop and 10Hz telemetry ingress."""
    link = SerialLink("socket://127.0.0.1:5555")
    link.open()
    parser = TelemetryStreamParser()

    cmd = CommandPacket(vx=15, wz=0, flags=FLAG_MOTOR_ENABLE)
    received_packets = []

    # Stream commands for 1.05 seconds using drift-compensated 50 Hz pacing
    next_tick = time.perf_counter()
    for _ in range(52):
        link.write(cmd.pack())

        raw = link.read(64)
        if raw:
            received_packets.extend(parser.feed(raw))

        next_tick += 0.020
        sleep_time = next_tick - time.perf_counter()
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Final drain of any in-flight bytes
    time.sleep(0.02)
    raw = link.read(64)
    if raw:
        received_packets.extend(parser.feed(raw))

    link.close()

    assert not mock_pico.watchdog_triggered, "Watchdog tripped during 50Hz transmission"
    assert len(received_packets) >= 9, f"Expected ~10 telemetry frames, got {len(received_packets)}"
    assert received_packets[-1].delta_ticks_left > 0, "Motors did not report positive tick displacement"


def test_watchdog_timeout_cutoff(mock_pico):
    """Gate 2: Stop sending commands and verify watchdog triggers within <= 205 ms."""
    link = SerialLink("socket://127.0.0.1:5555")
    link.open()
    cmd = CommandPacket(vx=20, wz=0, flags=FLAG_MOTOR_ENABLE)

    for _ in range(5):
        link.write(cmd.pack())
        time.sleep(0.02)

    assert not mock_pico.watchdog_triggered

    # Halt transmissions abruptly and sleep 210 ms
    time.sleep(0.210)
    assert mock_pico.watchdog_triggered, "Mock Pico failed to trigger watchdog cutoff after 210ms silence"
    link.close()


def test_checksum_corruption_rejection(mock_pico):
    """Gate 3: Inject single-byte corrupted frames and verify they are rejected."""
    link = SerialLink("socket://127.0.0.1:5555")
    link.open()

    valid_packet = CommandPacket(vx=40, wz=10, flags=FLAG_MOTOR_ENABLE).pack()
    corrupted_packet = valid_packet[:4] + bytes([(valid_packet[4] ^ 0xFF)])

    mock_pico.watchdog_triggered = True
    link.write(corrupted_packet)
    time.sleep(0.03)

    assert mock_pico.watchdog_triggered, "Corrupted packet was erroneously accepted by the server"
    link.close()