from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5 import *
from PyQt5.QtCore import QThread, pyqtSignal, QMutex, QRegularExpression, QObject, QTimer
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QRegularExpressionValidator, QDoubleValidator

from . import device_state
from . import serial_backend



class SleepTimer(QObject):
    """
    A countdown timer that emits time updates at fixed intervals using PyQt signals.

    This class implements a simple countdown timer based on `QTimer`. It periodically
    emits the remaining time (in seconds) until the countdown reaches zero. The timer
    is intended to be used in GUI applications where a background countdown must update
    a display or trigger an event when completed.

    Attributes
    ----------
    update_time_signal : pyqtSignal(float)
        Signal emitted with the remaining time in seconds (rounded to one decimal place)
        after each timer tick.
    remaining : float
        The remaining time in seconds. Initialized from the global variable `data_1`.
    timer : QTimer
        Internal Qt timer that triggers the `_tick` method every 100 milliseconds.

    Methods
    -------
    start()
        Starts the countdown timer.
    stop()
        Stops the countdown timer.
    _tick()
        Decrements the remaining time by 0.1 seconds per tick, emits updates via
        `update_time_signal`, and calls `device_state.running_time_flag_setter(0)`
        when the countdown reaches zero.

    Notes
    -----
    - The initial countdown value is taken from the global variable `data_1`.
    - Each tick occurs every 100 ms (0.1 s).
    - When the countdown completes, the timer stops automatically.
    """
    update_time_signal = pyqtSignal(float)  # emit float countdown values

    def __init__(self):
        super().__init__()
        self.worker_remaining = device_state.TxData()
        self.remaining = float(self.worker_remaining.time_acquisition) # get local_data_1 from global
        self.worker_flag_run_time = device_state.RunningTimeFlag()
        self.worker_reset_current_time = device_state.DownSampleSpecificFlag()
        self.timer = QTimer(self)
        self.timer.setInterval(100)  # 100 ms per tick
        self.timer.timeout.connect(self._tick)
        

    def start(self):
        self.timer.start()

    def stop(self):
        self.timer.stop()

    def _tick(self):
        self.remaining -= 0.1
        if self.remaining >= 0.0:
            self.update_time_signal.emit(round(self.remaining, 1))
        else:
            self.timer.stop()
            self.worker_flag_run_time.flag_running_time = False


class SocketThread(QThread):
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True

    def run(self):
        if self.running:
            serial_backend.thread_start()


    def stop(self):
        self.running = False