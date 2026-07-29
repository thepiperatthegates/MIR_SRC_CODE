# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'analyse_Window.ui'
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
from PySide6.QtWidgets import (QApplication, QComboBox, QGridLayout, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QMainWindow,
    QMenuBar, QPushButton, QRadioButton, QSizePolicy,
    QSplitter, QStatusBar, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget)

class Ui_analyse_Window(object):
    def setupUi(self, analyse_Window):
        if not analyse_Window.objectName():
            analyse_Window.setObjectName(u"analyse_Window")
        analyse_Window.resize(1524, 933)
        self.centralwidget = QWidget(analyse_Window)
        self.centralwidget.setObjectName(u"centralwidget")
        self.gridLayout_3 = QGridLayout(self.centralwidget)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.gridLayout_2 = QGridLayout()
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.label_file = QLabel(self.centralwidget)
        self.label_file.setObjectName(u"label_file")
        font = QFont()
        font.setFamilies([u"MS UI Gothic"])
        font.setPointSize(10)
        font.setBold(True)
        self.label_file.setFont(font)
        self.label_file.setAutoFillBackground(False)
        self.label_file.setStyleSheet(u"")

        self.gridLayout_2.addWidget(self.label_file, 1, 0, 1, 1)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.refresh_Button = QPushButton(self.centralwidget)
        self.refresh_Button.setObjectName(u"refresh_Button")
        self.refresh_Button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.refresh_Button.setStyleSheet(u"background-color: rgb(85, 170, 255);")

        self.horizontalLayout_2.addWidget(self.refresh_Button)

        self.csv_Button = QPushButton(self.centralwidget)
        self.csv_Button.setObjectName(u"csv_Button")
        self.csv_Button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))

        self.horizontalLayout_2.addWidget(self.csv_Button)

        self.splitter = QSplitter(self.centralwidget)
        self.splitter.setObjectName(u"splitter")
        self.splitter.setOrientation(Qt.Horizontal)
        self.label = QLabel(self.splitter)
        self.label.setObjectName(u"label")
        font1 = QFont()
        font1.setFamilies([u"Arial"])
        self.label.setFont(font1)
        self.splitter.addWidget(self.label)
        self.data_show_comboBox = QComboBox(self.splitter)
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.addItem("")
        self.data_show_comboBox.setObjectName(u"data_show_comboBox")
        self.data_show_comboBox.setCursor(QCursor(Qt.CursorShape.WhatsThisCursor))
        self.splitter.addWidget(self.data_show_comboBox)

        self.horizontalLayout_2.addWidget(self.splitter)

        self.save_Button = QPushButton(self.centralwidget)
        self.save_Button.setObjectName(u"save_Button")
        self.save_Button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))

        self.horizontalLayout_2.addWidget(self.save_Button)

        self.splitter_2 = QSplitter(self.centralwidget)
        self.splitter_2.setObjectName(u"splitter_2")
        self.splitter_2.setOrientation(Qt.Horizontal)
        self.layoutWidget = QWidget(self.splitter_2)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.gridLayout = QGridLayout(self.layoutWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.textbox_offset2 = QLineEdit(self.layoutWidget)
        self.textbox_offset2.setObjectName(u"textbox_offset2")

        self.gridLayout.addWidget(self.textbox_offset2, 1, 1, 1, 1)

        self.textbox_offset1 = QLineEdit(self.layoutWidget)
        self.textbox_offset1.setObjectName(u"textbox_offset1")

        self.gridLayout.addWidget(self.textbox_offset1, 1, 0, 1, 1)

        self.label_3 = QLabel(self.layoutWidget)
        self.label_3.setObjectName(u"label_3")
        font2 = QFont()
        font2.setFamilies([u"Arial"])
        font2.setPointSize(13)
        self.label_3.setFont(font2)

        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 1)

        self.label_4 = QLabel(self.layoutWidget)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setFont(font2)

        self.gridLayout.addWidget(self.label_4, 0, 1, 1, 1)

        self.take_Button = QRadioButton(self.layoutWidget)
        self.take_Button.setObjectName(u"take_Button")
        self.take_Button.setMinimumSize(QSize(79, 0))
        self.take_Button.setCursor(QCursor(Qt.CursorShape.PointingHandCursor))
        self.take_Button.setChecked(True)

        self.gridLayout.addWidget(self.take_Button, 0, 2, 2, 1)

        self.splitter_2.addWidget(self.layoutWidget)
        self.horizontalLayoutWidget = QWidget(self.splitter_2)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.splitter_2.addWidget(self.horizontalLayoutWidget)

        self.horizontalLayout_2.addWidget(self.splitter_2)


        self.gridLayout_2.addLayout(self.horizontalLayout_2, 0, 0, 1, 1)

        self.label_fr = QLabel(self.centralwidget)
        self.label_fr.setObjectName(u"label_fr")
        font3 = QFont()
        font3.setFamilies([u"Arial"])
        font3.setPointSize(14)
        self.label_fr.setFont(font3)

        self.gridLayout_2.addWidget(self.label_fr, 3, 0, 1, 1)

        self.mlp_layout = QVBoxLayout()
        self.mlp_layout.setObjectName(u"mlp_layout")
        self.table_Widget = QTableWidget(self.centralwidget)
        if (self.table_Widget.columnCount() < 13):
            self.table_Widget.setColumnCount(13)
        __qtablewidgetitem = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(2, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(3, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(4, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(5, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(6, __qtablewidgetitem6)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(7, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(8, __qtablewidgetitem8)
        __qtablewidgetitem9 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(9, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(10, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(11, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        self.table_Widget.setHorizontalHeaderItem(12, __qtablewidgetitem12)
        self.table_Widget.setObjectName(u"table_Widget")

        self.mlp_layout.addWidget(self.table_Widget)


        self.gridLayout_2.addLayout(self.mlp_layout, 2, 0, 1, 1)


        self.gridLayout_3.addLayout(self.gridLayout_2, 0, 0, 1, 1)

        analyse_Window.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(analyse_Window)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1524, 21))
        analyse_Window.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(analyse_Window)
        self.statusbar.setObjectName(u"statusbar")
        analyse_Window.setStatusBar(self.statusbar)

        self.retranslateUi(analyse_Window)

        QMetaObject.connectSlotsByName(analyse_Window)
    # setupUi

    def retranslateUi(self, analyse_Window):
        analyse_Window.setWindowTitle(QCoreApplication.translate("analyse_Window", u"MainWindow", None))
        self.label_file.setText("")
        self.refresh_Button.setText("")
        self.csv_Button.setText(QCoreApplication.translate("analyse_Window", u"Open CSV file", None))
        self.label.setText(QCoreApplication.translate("analyse_Window", u"Select", None))
        self.data_show_comboBox.setItemText(0, QCoreApplication.translate("analyse_Window", u"Data table", None))
        self.data_show_comboBox.setItemText(1, QCoreApplication.translate("analyse_Window", u"Currents diagram", None))
        self.data_show_comboBox.setItemText(2, QCoreApplication.translate("analyse_Window", u"Voltage diagram", None))
        self.data_show_comboBox.setItemText(3, QCoreApplication.translate("analyse_Window", u"Phase diagram", None))
        self.data_show_comboBox.setItemText(4, QCoreApplication.translate("analyse_Window", u"Angular velocity diagram", None))
        self.data_show_comboBox.setItemText(5, QCoreApplication.translate("analyse_Window", u"Torque diagram", None))
        self.data_show_comboBox.setItemText(6, QCoreApplication.translate("analyse_Window", u"Shear rate diagram", None))
        self.data_show_comboBox.setItemText(7, QCoreApplication.translate("analyse_Window", u"Shear stress diagram", None))
        self.data_show_comboBox.setItemText(8, QCoreApplication.translate("analyse_Window", u"Viscosity diagram", None))

        self.save_Button.setText(QCoreApplication.translate("analyse_Window", u"Save CSV file", None))
#if QT_CONFIG(tooltip)
        self.textbox_offset2.setToolTip("")
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        self.textbox_offset1.setToolTip("")
#endif // QT_CONFIG(tooltip)
        self.label_3.setText(QCoreApplication.translate("analyse_Window", u"Insert offset 1 (mA)", None))
        self.label_4.setText(QCoreApplication.translate("analyse_Window", u"Insert offset 2 (mA)", None))
        self.take_Button.setText(QCoreApplication.translate("analyse_Window", u"Input from main", None))
        self.label_fr.setText(QCoreApplication.translate("analyse_Window", u"fr1 =                                   fr0 = ", None))
        ___qtablewidgetitem = self.table_Widget.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("analyse_Window", u"Time / s", None))
        ___qtablewidgetitem1 = self.table_Widget.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("analyse_Window", u"Voltage 1 / V", None))
        ___qtablewidgetitem2 = self.table_Widget.horizontalHeaderItem(2)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("analyse_Window", u"Voltage 2 / V", None))
        ___qtablewidgetitem3 = self.table_Widget.horizontalHeaderItem(3)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("analyse_Window", u"Current 1 / mA", None))
        ___qtablewidgetitem4 = self.table_Widget.horizontalHeaderItem(4)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("analyse_Window", u"Current 2 / mA", None))
        ___qtablewidgetitem5 = self.table_Widget.horizontalHeaderItem(5)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("analyse_Window", u"Angle of permanent magnet \u03a6_B / rad", None))
        ___qtablewidgetitem6 = self.table_Widget.horizontalHeaderItem(6)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("analyse_Window", u"Angle of permanent magnet  \u03a6_m / rad", None))
        ___qtablewidgetitem7 = self.table_Widget.horizontalHeaderItem(7)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("analyse_Window", u"Phase difference / rad", None))
        ___qtablewidgetitem8 = self.table_Widget.horizontalHeaderItem(8)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("analyse_Window", u"Angular velocity / rad/s", None))
        ___qtablewidgetitem9 = self.table_Widget.horizontalHeaderItem(9)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("analyse_Window", u"Torque / Nm ", None))
        ___qtablewidgetitem10 = self.table_Widget.horizontalHeaderItem(10)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("analyse_Window", u"Shear rate / 1/s", None))
        ___qtablewidgetitem11 = self.table_Widget.horizontalHeaderItem(11)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("analyse_Window", u"Shear stress / Pa", None))
        ___qtablewidgetitem12 = self.table_Widget.horizontalHeaderItem(12)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("analyse_Window", u"Viscocity / Pa\u22c5 s ", None))
    # retranslateUi

