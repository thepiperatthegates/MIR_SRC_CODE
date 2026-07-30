"""
In-GUI console tab.
"""
import sys
from datetime import datetime

from PySide6.QtCore import QObject, Signal
from PySide6.QtGui import QFont, QTextCursor
from PySide6.QtWidgets import QPlainTextEdit


class StreamRedirector(QObject):
    """
    A fake "file" that print()/warnings can write text to.
    """

    message_written = Signal(str)

    encoding = "utf-8"

    def __init__(self, original_stream=None):
        super().__init__()
        self._original_stream = original_stream

    def write(self, text):
        """
        Called automatically by print(), warnings.warn(), traceback printing,
        etc. - anything that writes to sys.stdout or sys.stderr - because
        install() below has made sys.stdout/sys.stderr point at THIS object.
        """

        if self._original_stream is not None:
            try:
                self._original_stream.write(text)
            except Exception:
                pass

        if text:
            self.message_written.emit(text)

    def flush(self):
        """Some code calls sys.stdout.flush() explicitly - just pass it through."""
        if self._original_stream is not None:
            try:
                self._original_stream.flush()
            except Exception:
                pass

    def isatty(self):
        # Tells any code that checks "am I writing to a real terminal?" that
        # the answer is no. Some libraries change their formatting based on this.
        return False


class ConsoleWidget(QPlainTextEdit):
    """A read-only, black-background/green-text log box that displays everything
    written to the shared StreamRedirector (stdout or stderr)."""

    def __init__(self, stream, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(5000)  # cap history so it can't grow forever
        self.setFont(QFont("Consolas", 9))
        self.setStyleSheet("QPlainTextEdit { background-color: black; color: #00ff00; }")


        self._at_line_start = True

        stream.message_written.connect(self._append)

    def _append(self, text):
        """
        Runs on the GUI thread whenever the subscribed stream receives text.
        """
        self.moveCursor(QTextCursor.MoveOperation.End)

        lines = text.split("\n")

        for i, line in enumerate(lines):
            if i > 0:
                self.insertPlainText("\n")
                self._at_line_start = True

            if line:
                if self._at_line_start:
                    timestamp = datetime.now().strftime("%H:%M:%S")
                    self.insertPlainText(f"[{timestamp}] ")
                    self._at_line_start = False
                self.insertPlainText(line)

        self.moveCursor(QTextCursor.MoveOperation.End)


# These hold the two StreamRedirector instances once install() has run, so
# that every ConsoleWidget (one per experiment window) can attach to the SAME
# stream objects and therefore all show the same live output.
_stdout_stream = None
_stderr_stream = None


def install():
    """
    Replace sys.stdout and sys.stderr with our StreamRedirector objects.
    """
    global _stdout_stream, _stderr_stream

    if _stdout_stream is None:
        _stdout_stream = StreamRedirector(sys.stdout)
        _stderr_stream = StreamRedirector(sys.stderr)
        
        
        sys.stdout = _stdout_stream
        sys.stderr = _stderr_stream


def get_stdout_stream():
    """Lets other files (the TabWindow* classes) grab the shared stdout stream
    so they can hand it to a new ConsoleWidget."""
    return _stdout_stream


def get_stderr_stream():
    return _stderr_stream
