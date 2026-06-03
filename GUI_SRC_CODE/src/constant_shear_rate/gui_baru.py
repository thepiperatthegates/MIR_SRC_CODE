# -*- coding: utf-8 -*-
from PyQt5 import QtCore, QtGui, QtWidgets
from pyqtgraph import GraphicsLayoutWidget


class Ui_Title(object):
    def setupUi(self, Title):
        Title.setObjectName("Title")
        Title.resize(1307, 893)

        self.centralwidget = QtWidgets.QWidget(Title)
        self.centralwidget.setObjectName("centralwidget")

        root = QtWidgets.QHBoxLayout(self.centralwidget)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(12)

        # ── LEFT PANEL ───────────────────────────────────────────────
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(8)

        # GroupBox: Acquisition
        self.group_acquisition = QtWidgets.QGroupBox("Acquisition", self.centralwidget)
        self.group_acquisition.setObjectName("group_acquisition")
        acq = QtWidgets.QVBoxLayout(self.group_acquisition)
        acq.setSpacing(4)
        acq.setContentsMargins(8, 14, 8, 8)

        self.label_time = QtWidgets.QLabel(self.centralwidget)
        self.label_time.setObjectName("label_time")
        acq.addWidget(self.label_time)
        self.textbox_time = QtWidgets.QLineEdit(self.centralwidget)
        self.textbox_time.setObjectName("textbox_time")
        acq.addWidget(self.textbox_time)

        self.label_frequency = QtWidgets.QLabel(self.centralwidget)
        self.label_frequency.setObjectName("label_frequency")
        acq.addWidget(self.label_frequency)
        self.textbox_frequency = QtWidgets.QLineEdit(self.centralwidget)
        self.textbox_frequency.setObjectName("textbox_frequency")
        acq.addWidget(self.textbox_frequency)

        self.label_direction = QtWidgets.QLabel(self.centralwidget)
        self.label_direction.setObjectName("label_direction")
        acq.addWidget(self.label_direction)
        self.comboBox_direction = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_direction.setObjectName("comboBox_direction")
        self.comboBox_direction.addItem("")
        self.comboBox_direction.addItem("")
        acq.addWidget(self.comboBox_direction)

        self.label_sample_frequency = QtWidgets.QLabel(self.centralwidget)
        self.label_sample_frequency.setObjectName("label_sample_frequency")
        acq.addWidget(self.label_sample_frequency)
        self.textbox_sample_frequency = QtWidgets.QLineEdit(self.centralwidget)
        self.textbox_sample_frequency.setObjectName("textbox_sample_frequency")
        acq.addWidget(self.textbox_sample_frequency)

        left.addWidget(self.group_acquisition)

        # GroupBox: Motor 1
        self.group_motor1 = QtWidgets.QGroupBox("Coil 1", self.centralwidget)
        self.group_motor1.setObjectName("group_motor1")
        m1 = QtWidgets.QVBoxLayout(self.group_motor1)
        m1.setSpacing(4)
        m1.setContentsMargins(8, 14, 8, 8)

        self.label_amplitude_1 = QtWidgets.QLabel(self.centralwidget)
        self.label_amplitude_1.setObjectName("label_amplitude_1")
        m1.addWidget(self.label_amplitude_1)
        self.textbox_amplitude1 = QtWidgets.QLineEdit(self.centralwidget)
        self.textbox_amplitude1.setObjectName("textbox_amplitude1")
        m1.addWidget(self.textbox_amplitude1)

        self.label_offset_1 = QtWidgets.QLabel(self.centralwidget)
        self.label_offset_1.setObjectName("label_offset_1")
        m1.addWidget(self.label_offset_1)
        self.textbox_offset1 = QtWidgets.QLineEdit(self.centralwidget)
        self.textbox_offset1.setObjectName("textbox_offset1")
        m1.addWidget(self.textbox_offset1)

        left.addWidget(self.group_motor1)

        # GroupBox: Motor 2
        self.group_motor2 = QtWidgets.QGroupBox("Coil 2", self.centralwidget)
        self.group_motor2.setObjectName("group_motor2")
        m2 = QtWidgets.QVBoxLayout(self.group_motor2)
        m2.setSpacing(4)
        m2.setContentsMargins(8, 14, 8, 8)

        self.label_amplitude_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_amplitude_2.setObjectName("label_amplitude_2")
        m2.addWidget(self.label_amplitude_2)
        self.textbox_amplitude2 = QtWidgets.QLineEdit(self.centralwidget)
        self.textbox_amplitude2.setObjectName("textbox_amplitude2")
        m2.addWidget(self.textbox_amplitude2)

        self.label_offset_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_offset_2.setObjectName("label_offset_2")
        m2.addWidget(self.label_offset_2)
        self.textbox_offset2 = QtWidgets.QLineEdit(self.centralwidget)
        self.textbox_offset2.setObjectName("textbox_offset2")
        m2.addWidget(self.textbox_offset2)

        left.addWidget(self.group_motor2)

        left.addStretch(1)

        # 2×2 main control buttons
        btn_grid = QtWidgets.QGridLayout()
        btn_grid.setSpacing(6)

        self.button_send = QtWidgets.QPushButton(self.centralwidget)
        self.button_send.setObjectName("button_send")
        btn_grid.addWidget(self.button_send, 0, 0)

        self.button_stop = QtWidgets.QPushButton(self.centralwidget)
        self.button_stop.setObjectName("button_stop")
        self.button_stop.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        btn_grid.addWidget(self.button_stop, 0, 1)

        self.button_start = QtWidgets.QPushButton(self.centralwidget)
        self.button_start.setObjectName("button_start")
        self.button_start.setFocusPolicy(QtCore.Qt.NoFocus)
        btn_grid.addWidget(self.button_start, 1, 0)

        self.button_rotate = QtWidgets.QPushButton(self.centralwidget)
        self.button_rotate.setObjectName("button_rotate")
        self.button_rotate.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        btn_grid.addWidget(self.button_rotate, 1, 1)

        left.addLayout(btn_grid)

        self.save_button = QtWidgets.QPushButton(self.centralwidget)
        self.save_button.setObjectName("save_button")
        self.save_button.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        self.save_button.setText("")
        left.addWidget(self.save_button)

        # Status / LCD / interval row
        status_grid = QtWidgets.QGridLayout()
        status_grid.setSpacing(6)

        self.status_label = QtWidgets.QLabel(self.centralwidget)
        self.status_label.setObjectName("status_label")
        status_grid.addWidget(self.status_label, 0, 0, 1, 3)

        self.lcdNumber = QtWidgets.QLCDNumber(self.centralwidget)
        self.lcdNumber.setObjectName("lcdNumber")
        status_grid.addWidget(self.lcdNumber, 1, 0, 2, 2)

        self.label_time_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_time_2.setObjectName("label_time_2")
        status_grid.addWidget(self.label_time_2, 1, 2, 1, 1)

        self.timeInterval_comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.timeInterval_comboBox.setObjectName("timeInterval_comboBox")
        self.timeInterval_comboBox.addItem("")
        self.timeInterval_comboBox.addItem("")
        self.timeInterval_comboBox.addItem("")
        status_grid.addWidget(self.timeInterval_comboBox, 2, 2, 1, 1)

        left.addLayout(status_grid)

        self.graph_stop_button = QtWidgets.QPushButton(self.centralwidget)
        self.graph_stop_button.setObjectName("graph_stop_button")
        left.addWidget(self.graph_stop_button)

        self.button_fr_constant = QtWidgets.QPushButton(self.centralwidget)
        self.button_fr_constant.setObjectName("button_fr_constant")
        self.button_fr_constant.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        left.addWidget(self.button_fr_constant)

        self.button_cal_constant = QtWidgets.QPushButton(self.centralwidget)
        self.button_cal_constant.setObjectName("button_cal_constant")
        self.button_cal_constant.setCursor(QtGui.QCursor(QtCore.Qt.PointingHandCursor))
        left.addWidget(self.button_cal_constant)

        root.addLayout(left, 1)

        # ── RIGHT PANEL: graph ───────────────────────────────────────
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(6)

        title_row = QtWidgets.QHBoxLayout()

        self.TITLE = QtWidgets.QLabel(self.centralwidget)
        self.TITLE.setObjectName("TITLE")
        title_row.addWidget(self.TITLE)

        title_row.addStretch()

        self.select_mode_comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.select_mode_comboBox.setObjectName("select_mode_comboBox")
        self.select_mode_comboBox.addItem("")
        self.select_mode_comboBox.addItem("")
        self.select_mode_comboBox.addItem("")
        title_row.addWidget(self.select_mode_comboBox)

        right.addLayout(title_row)

        self.graphicsView = GraphicsLayoutWidget(self.centralwidget)
        self.graphicsView.setObjectName("graphicsView")
        right.addWidget(self.graphicsView, 1)

        self.k_b_label = QtWidgets.QLabel(self.centralwidget)
        self.k_b_label.setObjectName("k_b_label")
        right.addWidget(self.k_b_label)

        root.addLayout(right, 5)

        Title.setCentralWidget(self.centralwidget)

        # ── Menu / Status bar ────────────────────────────────────────
        self.menubar = QtWidgets.QMenuBar(Title)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1307, 22))
        self.menubar.setObjectName("menubar")
        self.menuRestart = QtWidgets.QMenu(self.menubar)
        self.menuRestart.setObjectName("menuRestart")
        self.flash_menu = QtWidgets.QMenu(self.menubar)
        self.flash_menu.setObjectName("flash_menu")
        Title.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(Title)
        self.statusbar.setObjectName("statusbar")
        Title.setStatusBar(self.statusbar)
        self.actionHardware_reset = QtWidgets.QAction(Title)
        self.actionHardware_reset.setObjectName("actionHardware_reset")
        self.actionSoftware_restart = QtWidgets.QAction(Title)
        self.actionSoftware_restart.setObjectName("actionSoftware_restart")
        self.actionFlash_STM32 = QtWidgets.QAction(Title)
        self.actionFlash_STM32.setObjectName("actionFlash_STM32")
        self.menuRestart.addAction(self.actionHardware_reset)
        self.menuRestart.addAction(self.actionSoftware_restart)
        self.flash_menu.addAction(self.actionFlash_STM32)
        self.menubar.addAction(self.menuRestart.menuAction())
        self.menubar.addAction(self.flash_menu.menuAction())

        self.retranslateUi(Title)
        QtCore.QMetaObject.connectSlotsByName(Title)

    def retranslateUi(self, Title):
        _translate = QtCore.QCoreApplication.translate
        Title.setWindowTitle(_translate("Title", "MainWindow"))
        self.TITLE.setText(_translate("Title", " Mini-Rheometer GUI"))
        self.k_b_label.setText(_translate("Title", ""))
        self.select_mode_comboBox.setItemText(0, _translate("Title", "View sensors"))
        self.select_mode_comboBox.setItemText(1, _translate("Title", "View angle"))
        self.select_mode_comboBox.setItemText(2, _translate("Title", "View phase difference"))
        self.label_time.setText(_translate("Title", "Time acquisition / s"))
        self.label_frequency.setText(_translate("Title", "Shear rate γ̇ / Hz"))
        self.label_amplitude_1.setText(_translate("Title", "Current amplitude 1 / mA"))
        self.label_offset_1.setText(_translate("Title", "Current offset 1 / mA"))
        self.label_amplitude_2.setText(_translate("Title", "Current amplitude 2 / mA"))
        self.label_offset_2.setText(_translate("Title", "Current offset 2 / mA"))
        self.label_direction.setText(_translate("Title", "Direction of rotation"))
        self.comboBox_direction.setItemText(0, _translate("Title", "Clockwise"))
        self.comboBox_direction.setItemText(1, _translate("Title", "Anti-clockwise"))
        self.label_sample_frequency.setText(_translate("Title", "Sample frequency / Hz"))
        self.button_rotate.setText(_translate("Title", "Rotate"))
        self.button_send.setText(_translate("Title", "Send data"))
        self.button_stop.setText(_translate("Title", "STOP!"))
        self.button_start.setText(_translate("Title", "Start acquisition"))
        self.timeInterval_comboBox.setItemText(0, _translate("Title", "500ms"))
        self.timeInterval_comboBox.setItemText(1, _translate("Title", "100ms"))
        self.timeInterval_comboBox.setItemText(2, _translate("Title", "1000ms"))
        self.button_fr_constant.setText(_translate("Title", "fᴿ rechnen"))
        self.label_time_2.setText(_translate("Title", "Time Interval (s)"))
        self.status_label.setText(_translate("Title", "Status : Ready"))
        self.graph_stop_button.setText(_translate("Title", "Pause live-graph"))
        self.button_cal_constant.setText(_translate("Title", "Calibrate Hall"))
        self.menuRestart.setTitle(_translate("Title", "Restart"))
        self.flash_menu.setTitle(_translate("Title", "FLASH"))
        self.actionHardware_reset.setText(_translate("Title", "Hardware reset"))
        self.actionSoftware_restart.setText(_translate("Title", "Software restart"))
        self.actionFlash_STM32.setText(_translate("Title", "Flash STM32"))
