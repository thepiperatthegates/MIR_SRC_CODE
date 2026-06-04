LIGHT_THEME = """
QToolTip
{
     border: 1px solid black;
     background-color: #ffa02f;
     padding: 1px;
     border-radius: 3px;
     opacity: 100;
}

QWidget
{
    color: #333333;
    background-color: #f5f5f5;
}

QLabel
{
    color: #333333;
    background-color: transparent;
}

QGroupBox
{
    color: #333333;
    border: 1px solid darkgray;
    margin-top: 10px;
}

QGroupBox::title
{
    color: #333333;
}

QGroupBox:focus
{
    border: 1px solid darkgray;
}

QTreeView, QListView
{
    background-color: #e8e8e8;
    margin-left: 5px;
}

QWidget:item:hover
{
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #ca0619);
    color: #000000;
}

QWidget:item:selected
{
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);
    color: #000000;
}

QMenuBar::item
{
    background: transparent;
}

QMenuBar::item:selected
{
    background: transparent;
    border: 1px solid #ffaa00;
}

QMenuBar::item:pressed
{
    background: #dddddd;
    border: 1px solid #aaaaaa;
    background-color: QLinearGradient(
        x1:0, y1:0,
        x2:0, y2:1,
        stop:0 #e8e8e8,
        stop:1 #d0d0d0
    );
    margin-bottom:-1px;
    padding-bottom:1px;
}

QMenu
{
    border: 1px solid #cccccc;
    background-color: #ffffff;
    color: #333333;
}

QMenu::item
{
    padding: 2px 20px 2px 20px;
    color: #333333;
}

QMenu::item:selected
{
    color: #000000;
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);
}

QWidget:disabled
{
    color: #aaaaaa;
    background-color: #f5f5f5;
}

QAbstractItemView
{
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffffff, stop: 0.1 #f5f5f5, stop: 1 #ebebeb);
    color: #333333;
}

QWidget:focus
{
    /*border: 1px solid darkgray;*/
}

QLineEdit
{
    background-color: #ffffff;
    color: #333333;
    padding: 1px;
    border-style: solid;
    border: 1px solid #cccccc;
    border-radius: 5;
}

QPushButton
{
    color: #333333;
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #e8e8e8, stop: 0.1 #e0e0e0, stop: 0.5 #d8d8d8, stop: 0.9 #d0d0d0, stop: 1 #c8c8c8);
    border-width: 1px;
    border-color: #aaaaaa;
    border-style: solid;
    border-radius: 6;
    padding: 3px;
    font-size: 12px;
    padding-left: 5px;
    padding-right: 5px;
    min-width: 40px;
}

QPushButton:pressed
{
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #b8b8b8, stop: 0.1 #bababa, stop: 0.5 #bcbcbc, stop: 0.9 #bebebe, stop: 1 #c0c0c0);
}

QComboBox
{
    color: #333333;
    selection-background-color: #ffaa00;
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #e8e8e8, stop: 0.1 #e0e0e0, stop: 0.5 #d8d8d8, stop: 0.9 #d0d0d0, stop: 1 #c8c8c8);
    border-style: solid;
    border: 1px solid #aaaaaa;
    border-radius: 5;
}

QComboBox:hover, QPushButton:hover
{
    border: 2px solid QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);
}

QComboBox:on
{
    padding-top: 3px;
    padding-left: 4px;
    background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #d0d0d0, stop: 0.1 #d2d2d2, stop: 0.5 #d4d4d4, stop: 0.9 #d6d6d6, stop: 1 #d8d8d8);
    selection-background-color: #ffaa00;
}

QComboBox QAbstractItemView
{
    border: 2px solid darkgray;
    background-color: #ffffff;
    color: #333333;
    selection-background-color: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);
}

QComboBox::drop-down
{
     subcontrol-origin: padding;
     subcontrol-position: top right;
     width: 15px;

     border-left-width: 0px;
     border-left-color: darkgray;
     border-left-style: solid;
     border-top-right-radius: 3px;
     border-bottom-right-radius: 3px;
}

QComboBox::down-arrow
{
     image: url(:/dark_orange/img/down_arrow.png);
}

QTextEdit:focus
{
    border: 1px solid darkgray;
}

QScrollBar:horizontal {
     border: 1px solid #cccccc;
     background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0.0 #e8e8e8, stop: 0.2 #e0e0e0, stop: 1 #d8d8d8);
     height: 7px;
     margin: 0px 16px 0 16px;
}

QScrollBar::handle:horizontal
{
      background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 #ffa02f, stop: 0.5 #d7801a, stop: 1 #ffa02f);
      min-height: 20px;
      border-radius: 2px;
}

QScrollBar::add-line:horizontal {
      border: 1px solid #cccccc;
      border-radius: 2px;
      background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 #ffa02f, stop: 1 #d7801a);
      width: 14px;
      subcontrol-position: right;
      subcontrol-origin: margin;
}

QScrollBar::sub-line:horizontal {
      border: 1px solid #cccccc;
      border-radius: 2px;
      background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0 #ffa02f, stop: 1 #d7801a);
      width: 14px;
     subcontrol-position: left;
     subcontrol-origin: margin;
}

QScrollBar::right-arrow:horizontal, QScrollBar::left-arrow:horizontal
{
      border: 1px solid black;
      width: 1px;
      height: 1px;
      background: white;
}

QScrollBar::add-page:horizontal, QScrollBar::sub-page:horizontal
{
      background: none;
}

QScrollBar:vertical
{
      background: QLinearGradient( x1: 0, y1: 0, x2: 1, y2: 0, stop: 0.0 #e8e8e8, stop: 0.2 #e0e0e0, stop: 1 #d8d8d8);
      width: 7px;
      margin: 16px 0 16px 0;
      border: 1px solid #cccccc;
}

QScrollBar::handle:vertical
{
      background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 0.5 #d7801a, stop: 1 #ffa02f);
      min-height: 20px;
      border-radius: 2px;
}

QScrollBar::add-line:vertical
{
      border: 1px solid #cccccc;
      border-radius: 2px;
      background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #ffa02f, stop: 1 #d7801a);
      height: 14px;
      subcontrol-position: bottom;
      subcontrol-origin: margin;
}

QScrollBar::sub-line:vertical
{
      border: 1px solid #cccccc;
      border-radius: 2px;
      background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #d7801a, stop: 1 #ffa02f);
      height: 14px;
      subcontrol-position: top;
      subcontrol-origin: margin;
}

QScrollBar::up-arrow:vertical, QScrollBar::down-arrow:vertical
{
      border: 1px solid black;
      width: 1px;
      height: 1px;
      background: white;
}

QScrollBar::add-page:vertical, QScrollBar::sub-page:vertical
{
      background: none;
}

QTextEdit
{
    background-color: #ffffff;
    color: #333333;
}

QPlainTextEdit
{
    background-color: #ffffff;
    color: #333333;
}

QHeaderView::section
{
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e0e0e0, stop: 0.5 #d4d4d4, stop: 0.6 #cccccc, stop:1 #e0e0e0);
    color: #333333;
    padding-left: 4px;
    border: 1px solid #bbbbbb;
}

QCheckBox:disabled
{
    color: #aaaaaa;
}

QDockWidget::title
{
    text-align: center;
    spacing: 3px;
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e0e0e0, stop: 0.5 #d4d4d4, stop:1 #e0e0e0);
}

QDockWidget::close-button, QDockWidget::float-button
{
    text-align: center;
    spacing: 1px;
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e0e0e0, stop: 0.5 #d4d4d4, stop:1 #e0e0e0);
}

QDockWidget::close-button:hover, QDockWidget::float-button:hover
{
    background: #d4d4d4;
}

QDockWidget::close-button:pressed, QDockWidget::float-button:pressed
{
    padding: 1px -1px -1px 1px;
}

QMainWindow::separator
{
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e0e0e0, stop: 0.5 #d8d8d8, stop: 0.6 #d0d0d0, stop:1 #e0e0e0);
    color: #333333;
    padding-left: 4px;
    border: 1px solid #bbbbbb;
    spacing: 3px;
}

QMainWindow::separator:hover
{
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #d7801a, stop:0.5 #b56c17 stop:1 #ffa02f);
    color: white;
    padding-left: 4px;
    border: 1px solid #6c6c6c;
    spacing: 3px;
}

QToolBar::handle
{
     spacing: 3px;
     background: url(:/dark_orange/img/handle.png);
}

QMenu::separator
{
    height: 2px;
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #e0e0e0, stop: 0.5 #d8d8d8, stop: 0.6 #d0d0d0, stop:1 #e0e0e0);
    color: #333333;
    padding-left: 4px;
    margin-left: 10px;
    margin-right: 5px;
}

QProgressBar
{
    border: 2px solid grey;
    border-radius: 5px;
    text-align: center;
    color: #333333;
}

QProgressBar::chunk
{
    background-color: #d7801a;
    width: 2.15px;
    margin: 0.5px;
}

QTabBar::tab {
    color: #333333;
    border: 1px solid #bbbbbb;
    border-bottom-style: none;
    background-color: #e0e0e0;
    padding-left: 10px;
    padding-right: 10px;
    padding-top: 3px;
    padding-bottom: 2px;
    margin-right: -1px;
}

QTabWidget::pane {
    border: 1px solid #222222;
    top: 1px;
}

QTabBar::tab:last
{
    margin-right: 0;
    border-top-right-radius: 3px;
}

QTabBar::tab:first:!selected
{
    margin-left: 0px;
    border-top-left-radius: 3px;
}

QTabBar::tab:!selected
{
    color: #555555;
    border-bottom-style: solid;
    margin-top: 3px;
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:1 #d4d4d4, stop:.4 #e0e0e0);
}

QTabBar::tab:selected
{
    border-top-left-radius: 3px;
    border-top-right-radius: 3px;
    margin-bottom: 0px;
}

QTabBar::tab:!selected:hover
{
    border-top-left-radius: 3px;
    border-top-right-radius: 3px;
    background-color: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:1 #d4d4d4, stop:0.4 #e0e0e0, stop:0.2 #e0e0e0, stop:0.1 #ffaa00);
}

QRadioButton::indicator:checked, QRadioButton::indicator:unchecked{
    color: #333333;
    background-color: #ffffff;
    border: 1px solid #888888;
    border-radius: 6px;
}

QRadioButton::indicator:checked
{
    background-color: qradialgradient(
        cx: 0.5, cy: 0.5,
        fx: 0.5, fy: 0.5,
        radius: 1.0,
        stop: 0.25 #ffaa00,
        stop: 0.3 #ffffff
    );
}

QCheckBox::indicator{
    color: #333333;
    background-color: #ffffff;
    border: 1px solid #888888;
    width: 9px;
    height: 9px;
}

QRadioButton::indicator
{
    border-radius: 6px;
}

QRadioButton::indicator:hover, QCheckBox::indicator:hover
{
    border: 1px solid #ffaa00;
}

QCheckBox::indicator:checked
{
    image:url(:/dark_orange/img/checkbox.png);
}

QCheckBox::indicator:disabled, QRadioButton::indicator:disabled
{
    border: 1px solid #bbbbbb;
}

QSlider::groove:horizontal {
    border: 1px solid #bbbbbb;
    height: 8px;
    background: #e0e0e0;
    margin: 2px 0;
    border-radius: 2px;
}

QSlider::handle:horizontal {
    background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1,
      stop: 0.0 silver, stop: 0.2 #a8a8a8, stop: 1 #727272);
    border: 1px solid #aaaaaa;
    width: 14px;
    height: 14px;
    margin: -4px 0;
    border-radius: 2px;
}

QSlider::groove:vertical {
    border: 1px solid #bbbbbb;
    width: 8px;
    background: #e0e0e0;
    margin: 0 0px;
    border-radius: 2px;
}

QSlider::handle:vertical {
    background: QLinearGradient( x1: 0, y1: 0, x2: 0, y2: 1, stop: 0.0 silver,
      stop: 0.2 #a8a8a8, stop: 1 #727272);
    border: 1px solid #aaaaaa;
    width: 14px;
    height: 14px;
    margin: 0 -4px;
    border-radius: 2px;
}

QLCDNumber
{
    background-color: #000000;
    color: #00dd00;
    border: 1px solid #444444;
    border-radius: 2px;
}

QAbstractSpinBox {
    padding-top: 2px;
    padding-bottom: 2px;
    border: 1px solid darkgray;
    background-color: #ffffff;
    color: #333333;
    border-radius: 2px;
    min-width: 50px;
}
"""
