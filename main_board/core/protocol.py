"""
Binary UART protocol framing, serialization, and stream parsing.
Adheres strictly to docs/contracts/uart_protocol_contract.md.
"""

from dataclasses import dataclass
import struct
from typing import Optional, List, Tuple


# Protocol Constants
CMD_START_SENTINEL = 0xAA
CMD_FRAME_SIZE = 5

TEL_START_SENTINEL = 0x55
TEL_FRAME_SIZE = 8

# Control Flags Bitmask (Byte 3 of Command Frame)
FLAG_MOTOR_ENABLE = 0x01      # Bit 0: 1 = Active, 0 = Standby / E-Stop
FLAG_EMERGENCY_FALL = 0x02    # Bit 1: 1 = Hard Lock Motors
FLAG_DIALOGUE_ACTIVE = 0x04   # Bit 2: 1 = Hold Velocity Smoothly

# Telemetry Byte 6 Masks
TEL_BATTERY_MASK = 0x7F       # Bits 0..6: 0-100%
TEL_STALL_FLAG_BIT = 0x80     # Bit 7: 1 = Stalled, 0 = OK


@dataclass(frozen=True)
class CommandPacket:
    vx: int          # -100 to +100 cm/s
    wz: int          # -128 to +127 deg/s (positive = CCW)
    flags: int = FLAG_MOTOR_ENABLE

    def pack(self) -> bytes:
        """Serializes command into strict 5-byte binary frame."""
        clamped_vx = max(-100, min(100, int(self.vx)))
        clamped_wz = max(-128, min(127, int(self.wz)))
        flags = self.flags & 0xFF
        checksum = (clamped_vx + clamped_wz + flags) & 0xFF
        return struct.pack(">BbbBB", CMD_START_SENTINEL, clamped_vx, clamped_wz, flags, checksum)

    @classmethod
    def unpack(cls, data: bytes) -> Optional["CommandPacket"]:
        """Parses and validates a 5-byte command buffer."""
        if len(data) != CMD_FRAME_SIZE:
            return None
        sentinel, vx, wz, flags, cs = struct.unpack(">BbbBB", data)
        if sentinel != CMD_START_SENTINEL:
            return None
        expected_cs = (vx + wz + flags) & 0xFF
        if cs != expected_cs:
            return None
        return cls(vx=vx, wz=wz, flags=flags)


@dataclass(frozen=True)
class TelemetryPacket:
    pitch: int            # -90 to +90 deg
    roll: int             # -90 to +90 deg
    delta_ticks_left: int # Signed encoder ticks over 100ms
    delta_ticks_right: int# Signed encoder ticks over 100ms
    sonar_dist_cm: int    # 0 to 255 cm
    battery_pct: int      # 0 to 100%
    is_stalled: bool      # True if current stall detected

    def pack(self) -> bytes:
        """Constructs an 8-byte telemetry frame with Byte 6 bit-packing."""
        packed_byte6 = (max(0, min(100, self.battery_pct)) & TEL_BATTERY_MASK)
        if self.is_stalled:
            packed_byte6 |= TEL_STALL_FLAG_BIT

        payload = struct.pack(
            ">bbbbBB",
            self.pitch,
            self.roll,
            self.delta_ticks_left,
            self.delta_ticks_right,
            self.sonar_dist_cm,
            packed_byte6,
        )
        checksum = sum(payload) & 0xFF
        return struct.pack(">B", TEL_START_SENTINEL) + payload + struct.pack(">B", checksum)

    @classmethod
    def unpack(cls, data: bytes) -> Optional["TelemetryPacket"]:
        """Validates and unpacks an 8-byte telemetry frame."""
        if len(data) != TEL_FRAME_SIZE or data[0] != TEL_START_SENTINEL:
            return None
        expected_cs = sum(data[1:7]) & 0xFF
        if data[7] != expected_cs:
            return None

        pitch, roll, dl, dr, sonar, byte6 = struct.unpack(">bbbbBB", data[1:7])
        battery = byte6 & TEL_BATTERY_MASK
        is_stalled = bool(byte6 & TEL_STALL_FLAG_BIT)
        return cls(
            pitch=pitch,
            roll=roll,
            delta_ticks_left=dl,
            delta_ticks_right=dr,
            sonar_dist_cm=sonar,
            battery_pct=battery,
            is_stalled=is_stalled,
        )


class TelemetryStreamParser:
    """
    Non-blocking sliding-window parser. Handles fragmented TCP/UART streams
    without falling out of frame sync when corrupted bytes are encountered.
    """
    def __init__(self):
        self._buffer = bytearray()

    def feed(self, raw_bytes: bytes) -> List[TelemetryPacket]:
        self._buffer.extend(raw_bytes)
        packets: List[TelemetryPacket] = []

        while len(self._buffer) >= TEL_FRAME_SIZE:
            # Locate synchronization sentinel
            if self._buffer[0] != TEL_START_SENTINEL:
                self._buffer.pop(0)
                continue

            frame_candidate = bytes(self._buffer[:TEL_FRAME_SIZE])
            packet = TelemetryPacket.unpack(frame_candidate)
            if packet is not None:
                packets.append(packet)
                del self._buffer[:TEL_FRAME_SIZE]
            else:
                # Corrupted candidate or false sentinel; advance by 1 byte to resync
                self._buffer.pop(0)

        return packets