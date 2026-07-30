
from PySide6.QtWidgets import QMainWindow, QTabWidget
import multiprocessing
import os

import serial_comm.serial_backend as serial_backend
from .constant_shear_rate_gui import ConstShearGUI
from .analyse.backend import AnalyseWindow
from .graph_fr import PlotWindow
import console.console_widget as console_widget
from console.console_widget import ConsoleWidget
from console.serial_terminal_widget import SerialTerminalWidget


#different windows with tab
class TabWindowConstSR(QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle("MiR | Mini-Rheometer")
        self.resize(1600, 980)  # width, height


        # Create tab widget
        tabs = QTabWidget()

        # Add your classes as tabs
        tabs.addTab(ConstShearGUI(), "Main GUI")
        tabs.addTab( AnalyseWindow(), "Data analyse")
        tabs.addTab(PlotWindow(), "f_R Plot")
        tabs.addTab(ConsoleWidget(console_widget.get_stdout_stream()), "Python Console")
        self.serial_terminal = SerialTerminalWidget()
        tabs.addTab(self.serial_terminal, "Serial Terminal")

        # Set central widget
        self.setCentralWidget(tabs)

    def closeEvent(self, event):
        #stop all the background processes

        self.serial_terminal.stop_reader()

        for q in (serial_backend.q_to_process, serial_backend.q_to_graph, serial_backend.q_to_csv):
            q.close()
            q.join_thread()


        #terminate the other subprocess (may never have started if the device never connected)
        if serial_backend.p1 is not None:
            serial_backend.p1.terminate()
            serial_backend.p1.join()

        #stop all the threads

        self.main_window = ConstShearGUI()
        self.main_window.worker_socket.stop()
        self.main_window.worker_DataUpdate.stop()

        #DELETE THE GODDAMN FILE
        try:
            os.remove("dummy.csv")
        except OSError as e:
            print(f"Error deleting file: {e}")

        event.accept()



if __name__ == '__main__':
    multiprocessing.freeze_support()

    csr_window = TabWindowConstSR()
    csr_window.showMaximized()
