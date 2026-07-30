from PySide6.QtWidgets import QMainWindow, QTabWidget
import os

from serial_comm import serial_backend as sockets_files
from .creep_test_backend import CreepTestGUI
from .analyse_window_creep_test import AnalyseWindow
import console.console_widget as console_widget
from console.console_widget import ConsoleWidget
from console.serial_terminal_widget import SerialTerminalWidget


class TabWindowCreepTest(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR | Mini-Rheometer")


        # Create tab widget
        tabs = QTabWidget()

        # Add your classes as tabs
        tabs.addTab(CreepTestGUI(), "Main GUI")
        tabs.addTab( AnalyseWindow(), "Data analyse")
        tabs.addTab(ConsoleWidget(console_widget.get_stdout_stream()), "Console")
        self.serial_terminal = SerialTerminalWidget()
        tabs.addTab(self.serial_terminal, "Serial Terminal")

        # Set central widget
        self.setCentralWidget(tabs)


    def closeEvent(self, event):
        #stop all the background processes

        self.serial_terminal.stop_reader()

        for q in (sockets_files.q_to_process, sockets_files.q_to_graph, sockets_files.q_to_csv):
            q.close()
            q.join_thread()


        #terminate the other subprocess (may never have started if the device never connected)
        if sockets_files.p1 is not None:
            sockets_files.p1.terminate()
            sockets_files.p1.join()

        #stop all the threads

        self.main_window = CreepTestGUI()
        self.main_window.worker_socket.stop()
        self.main_window.worker_DataUpdate.stop()

        #DELETE THE GODDAMN FILE
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")

        event.accept()
