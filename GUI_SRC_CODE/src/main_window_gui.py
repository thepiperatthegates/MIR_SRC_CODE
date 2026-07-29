# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'main_window_gui.ui'
##
## Created by: Qt User Interface Compiler version 6.11.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QGridLayout, QLabel,
    QMainWindow, QMenuBar, QSizePolicy, QStatusBar,
    QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(501, 319)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.photo_label = QLabel(self.centralwidget)
        self.photo_label.setObjectName(u"photo_label")
        self.photo_label.setGeometry(QRect(20, 20, 181, 51))
        self.photo_label.setScaledContents(False)
        self.layoutWidget = QWidget(self.centralwidget)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(80, 100, 352, 76))
        self.gridLayout = QGridLayout(self.layoutWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.label_choose_experiment = QLabel(self.layoutWidget)
        self.label_choose_experiment.setObjectName(u"label_choose_experiment")
        font = QFont()
        font.setFamilies([u"Arial"])
        font.setPointSize(14)
        self.label_choose_experiment.setFont(font)

        self.gridLayout.addWidget(self.label_choose_experiment, 0, 2, 1, 1)

        self.choose_electronic_comboBox = QComboBox(self.layoutWidget)
        self.choose_electronic_comboBox.addItem("")
        self.choose_electronic_comboBox.addItem("")
        self.choose_electronic_comboBox.addItem("")
        self.choose_electronic_comboBox.setObjectName(u"choose_electronic_comboBox")
        font1 = QFont()
        font1.setBold(True)
        self.choose_electronic_comboBox.setFont(font1)

        self.gridLayout.addWidget(self.choose_electronic_comboBox, 1, 0, 1, 2)

        self.label_choose_electronic = QLabel(self.layoutWidget)
        self.label_choose_electronic.setObjectName(u"label_choose_electronic")
        self.label_choose_electronic.setFont(font)

        self.gridLayout.addWidget(self.label_choose_electronic, 0, 0, 1, 2)

        self.choose_experiment_comboBox = QComboBox(self.layoutWidget)
        self.choose_experiment_comboBox.addItem("")
        self.choose_experiment_comboBox.addItem("")
        self.choose_experiment_comboBox.addItem("")
        self.choose_experiment_comboBox.addItem("")
        self.choose_experiment_comboBox.setObjectName(u"choose_experiment_comboBox")
        self.choose_experiment_comboBox.setFont(font1)

        self.gridLayout.addWidget(self.choose_experiment_comboBox, 1, 2, 1, 1)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 501, 21))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"MainWindow", None))
        self.photo_label.setText("")
        self.label_choose_experiment.setText(QCoreApplication.translate("MainWindow", u"Types of experiment ", None))
        self.choose_electronic_comboBox.setItemText(0, QCoreApplication.translate("MainWindow", u"Choose now", None))
        self.choose_electronic_comboBox.setItemText(1, QCoreApplication.translate("MainWindow", u"Electronic 1", None))
        self.choose_electronic_comboBox.setItemText(2, QCoreApplication.translate("MainWindow", u"Electronic 2", None))

        self.label_choose_electronic.setText(QCoreApplication.translate("MainWindow", u"Choose Electronics", None))
        self.choose_experiment_comboBox.setItemText(0, QCoreApplication.translate("MainWindow", u"Choose experiment", None))
        self.choose_experiment_comboBox.setItemText(1, QCoreApplication.translate("MainWindow", u"Control shear rate", None))
        self.choose_experiment_comboBox.setItemText(2, QCoreApplication.translate("MainWindow", u"MAPHEUS", None))
        self.choose_experiment_comboBox.setItemText(3, QCoreApplication.translate("MainWindow", u"Pid-Controller (dev)", None))

    # retranslateUi

