"""
Read-only serial terminal tab: streams raw output from a specific USB serial
device (identified by VID/PID, not by COM port number, since that can change
between reboots/replugs) straight into a Qt text widget.

This is separate from serial_comm/serial_backend.py, which talks to the main
data-acquisition board (VID_03FD&PID_0100) using a structured binary protocol.
This widget instead just displays whatever raw bytes a DIFFERENT device sends
(VID_04B4&PID_0008, baud 115200) - useful for watching a secondary debug/console
UART without needing to open a separate terminal program.
"""
import time

import serial
import serial.tools.list_ports
from PySide6.QtCore import QThread, Signal
from PySide6.QtGui import QFont, QTextCursor
from PySide6.QtWidgets import QPlainTextEdit


def find_serial_port(vid, pid):
    """Return the COM port name matching the given USB VID/PID, or None if not plugged in."""
    for port_info in serial.tools.list_ports.comports():
        if port_info.vid == vid and port_info.pid == pid:
            return port_info.device
    return None


class SerialTerminalReader(QThread):
    """
    Background thread that keeps trying to connect to the target USB device
    and streams whatever it sends as decoded text via the data_received signal.

    Runs forever (until stop() is called): if the device isn't plugged in yet,
    or gets unplugged mid-session, it just keeps retrying rather than giving up
    - this is a passive monitor tab, not a critical data path.
    """

    data_received = Signal(str)

    def __init__(self, vid, pid, baudrate, parent=None):
        super().__init__(parent)
        self.vid = vid
        self.pid = pid
        self.baudrate = baudrate
        self._running = True

    def stop(self):
        self._running = False

    def run(self):
        while self._running:
            port_name = find_serial_port(self.vid, self.pid)
            if port_name is None:
                self.data_received.emit(
                    f"Waiting for device (VID_{self.vid:04X}&PID_{self.pid:04X})...\n"
                )
                time.sleep(1.0)
                continue

            try:
                with serial.Serial(port_name, self.baudrate, timeout=0.2) as ser:
                    self.data_received.emit(f"Connected on {port_name} @ {self.baudrate} baud\n")
                    while self._running:
                        chunk = ser.read(ser.in_waiting or 1)
                        if chunk:
                            self.data_received.emit(chunk.decode("utf-8", errors="replace"))
            except (serial.SerialException, OSError) as e:
                self.data_received.emit(f"Serial error: {e}\n")
                time.sleep(1.0)


class SerialTerminalWidget(QPlainTextEdit):
    """Read-only live view of a raw serial device, styled to match the Console tab."""

    VID = 0x04B4
    PID = 0x0008
    BAUD_RATE = 115200

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(5000)
        self.setFont(QFont("Consolas", 9))
        self.setStyleSheet("QPlainTextEdit { background-color: black; color: #00ff00; }")

        self._reader = SerialTerminalReader(self.VID, self.PID, self.BAUD_RATE, self)
        self._reader.data_received.connect(self._append)
        self._reader.start()

    def _append(self, text):
        self.moveCursor(QTextCursor.MoveOperation.End)
        self.insertPlainText(text)
        self.moveCursor(QTextCursor.MoveOperation.End)

    def stop_reader(self):
        """Stop the background reader thread. Call this from the parent window's
        closeEvent so the thread doesn't get destroyed while still running."""
        self._reader.stop()
        self._reader.wait(1000)
