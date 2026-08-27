from PySide6.QtWidgets import QMainWindow, QTabWidget
import os

from serial_comm import serial_backend as sockets_files
from .creep_test_backend import CreepTestGUI
from .analyse_window_creep_test import AnalyseWindow
import console.console_widget as console_widget
from console.console_widget import ConsoleWidget


class TabWindowCreepTest(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR | Mini-Rheometer")


        # Create tab widget
        tabs = QTabWidget()

        # Add your classes as tabs
        self.main_window = CreepTestGUI()
        tabs.addTab(self.main_window, "Main GUI")
        tabs.addTab( AnalyseWindow(), "Data analyse")
        tabs.addTab(ConsoleWidget(console_widget.get_stdout_stream()), "Console")

        # Set central widget
        self.setCentralWidget(tabs)


    def closeEvent(self, event):
        #stop all the background processes

        #stop the real background threads *before* closing the queues they
        #read from -- q_to_graph.get() blocks with no timeout, so setting
        #the flag alone doesn't unblock it; push a sentinel to wake it, then
        #wait for the thread to actually exit.
        self.main_window.worker_socket.stop()
        self.main_window.worker_DataUpdate.stop()
        sockets_files.q_to_graph.put(None)
        self.main_window.worker_DataUpdate.wait()

        #worker_socket.run() calls serial_backend.thread_start(), which
        #blocks forever in "while True: tx_event.wait()" and never checks
        #self.running -- .stop() above is a no-op for it, so it can't exit
        #on its own. Force it since the process is exiting anyway; this is
        #what was causing "QThread: Destroyed while thread is still running".
        self.main_window.worker_socket.terminate()
        self.main_window.worker_socket.wait()

        for q in (sockets_files.q_to_process, sockets_files.q_to_graph, sockets_files.q_to_csv, sockets_files.q_to_watchdog):
            q.close()
            q.join_thread()


        #terminate the other subprocess (may never have started if the device never connected)
        if sockets_files.p1 is not None:
            sockets_files.p1.terminate()
            sockets_files.p1.join()

        #DELETE THE GODDAMN FILE
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")

        event.accept()
