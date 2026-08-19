# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'MAMPHEUS_photo_tab.ui'
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
from PySide6.QtWidgets import (QApplication, QGridLayout, QLabel, QMainWindow,
    QPushButton, QSizePolicy, QWidget)

class Ui_analyse_Window(object):
    def setupUi(self, analyse_Window):
        if not analyse_Window.objectName():
            analyse_Window.setObjectName(u"analyse_Window")
        analyse_Window.resize(452, 405)
        self.centralwidget = QWidget(analyse_Window)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout = QGridLayout(self.centralwidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.refresh_Button = QPushButton(self.centralwidget)
        self.refresh_Button.setObjectName(u"refresh_Button")
        self.refresh_Button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.refresh_Button.setStyleSheet(u"background-color: rgb(85, 170, 255);")

        self.gridLayout.addWidget(self.refresh_Button, 0, 0, 1, 1)

        self.csv_Button = QPushButton(self.centralwidget)
        self.csv_Button.setObjectName(u"csv_Button")
        self.csv_Button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))

        self.gridLayout.addWidget(self.csv_Button, 0, 1, 1, 1)

        self.image_Label = QLabel(self.centralwidget)
        self.image_Label.setObjectName(u"image_Label")
        self.image_Label.setMinimumSize(QSize(420, 240))
        self.image_Label.setStyleSheet(u"background-color: rgb(30,30,30); border: 1px solid gray;")
        self.image_Label.setAlignment(Qt.AlignCenter)
        self.image_Label.setScaledContents(True)

        self.gridLayout.addWidget(self.image_Label, 1, 0, 1, 2)

        analyse_Window.setCentralWidget(self.centralwidget)

        self.retranslateUi(analyse_Window)

        QMetaObject.connectSlotsByName(analyse_Window)
    # setupUi

    def retranslateUi(self, analyse_Window):
        analyse_Window.setWindowTitle(QCoreApplication.translate("analyse_Window", u"MainWindow", None))
        self.refresh_Button.setText("")
        self.csv_Button.setText(QCoreApplication.translate("analyse_Window", u"Open .bin", None))
    # retranslateUi

