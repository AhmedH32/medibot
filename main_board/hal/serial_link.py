"""
Platform-agnostic non-blocking serial transport layer.
Seamlessly routes across TCP sockets (laptop simulation) and /dev/ttyAMA0 (Pi 5).
"""

import logging
import socket
from typing import Optional
import serial

logger = logging.getLogger("MediBot.HAL.Serial")


class SerialLink:
    def __init__(self, port_url: str, baudrate: int = 115200, timeout_s: float = 0.0):
        self.port_url = port_url
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._handle: Optional[serial.Serial] = None

    def open(self) -> None:
        """Opens physical UART or establishes low-latency TCP loopback socket."""
        if self._handle and self._handle.is_open:
            return

        logger.info(f"Connecting serial link via: {self.port_url}")
        # timeout=0.0 enforces non-blocking reads on both POSIX UART and TCP sockets
        self._handle = serial.serial_for_url(
            self.port_url,
            baudrate=self.baudrate,
            timeout=0.0,
            write_timeout=0.01,
        )

        # Disable Nagle's algorithm when running over TCP socket simulation
        raw_sock = getattr(self._handle, "_socket", None)
        if raw_sock is not None:
            raw_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self._handle.reset_input_buffer()
        self._handle.reset_output_buffer()

    def write(self, data: bytes) -> int:
        if not self.is_open():
            raise ConnectionError(f"Cannot write to closed port: {self.port_url}")
        return self._handle.write(data)

    def read(self, max_bytes: int = 64) -> bytes:
        """Drains up to max_bytes available in the OS buffer without blocking."""
        if not self.is_open():
            return b""
        try:
            return self._handle.read(max_bytes)
        except Exception:
            return b""

    def is_open(self) -> bool:
        return self._handle is not None and self._handle.is_open

    def close(self) -> None:
        if self._handle and self._handle.is_open:
            self._handle.close()
            logger.info("Serial transport closed.")
        self._handle = None