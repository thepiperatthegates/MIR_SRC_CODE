LIGHT_THEME = """
/* ── Base ──────────────────────────────────────────────────── */
QMainWindow, QWidget {
    background-color: #f0f2f5;
    color: #1f2937;
    font-family: "Segoe UI", Arial, sans-serif;
    font-size: 12px;
}

/* ── GroupBox ───────────────────────────────────────────────── */
QGroupBox {
    background-color: #ffffff;
    border: 1px solid #d1d5db;
    border-radius: 6px;
    margin-top: 10px;
    padding: 6px 4px 4px 4px;
    font-weight: bold;
    color: #1d4ed8;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 10px;
    padding: 0 4px;
}

/* ── Labels ─────────────────────────────────────────────────── */
QLabel { color: #1f2937; }

#TITLE {
    color: #1d4ed8;
    font-size: 20px;
    font-weight: bold;
}
#status_label {
    color: #15803d;
    font-size: 13px;
    font-weight: bold;
}
#k_b_label { color: #6b7280; font-size: 11px; }

/* ── Line Edits ─────────────────────────────────────────────── */
QLineEdit {
    background-color: #ffffff;
    color: #1f2937;
    border: 1px solid #d1d5db;
    border-radius: 4px;
    padding: 4px 6px;
    selection-background-color: #bfdbfe;
}
QLineEdit:focus { border: 1px solid #2563eb; }

/* ── ComboBox ───────────────────────────────────────────────── */
QComboBox {
    background-color: #ffffff;
    color: #1f2937;
    border: 1px solid #d1d5db;
    border-radius: 4px;
    padding: 4px 8px;
}
QComboBox:hover { border: 1px solid #2563eb; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox QAbstractItemView {
    background-color: #ffffff;
    color: #1f2937;
    selection-background-color: #dbeafe;
    outline: none;
}

/* ── Buttons (base) ─────────────────────────────────────────── */
QPushButton {
    background-color: #e5e7eb;
    color: #1f2937;
    border: 1px solid #d1d5db;
    border-radius: 5px;
    padding: 6px 12px;
    font-weight: 600;
    min-height: 28px;
}
QPushButton:hover    { background-color: #d1d5db; }
QPushButton:pressed  { background-color: #9ca3af; }
QPushButton:disabled { background-color: #f3f4f6; color: #9ca3af; }

/* ── Semantic buttons ───────────────────────────────────────── */
#button_start               { background-color: #16a34a; color: #fff; border: none; }
#button_start:hover         { background-color: #15803d; }
#button_start:disabled      { background-color: #bbf7d0; color: #6b7280; }

#button_stop                { background-color: #dc2626; color: #fff; border: none; font-weight: bold; }
#button_stop:hover          { background-color: #b91c1c; }

#button_send                { background-color: #2563eb; color: #fff; border: none; }
#button_send:hover          { background-color: #1d4ed8; }

#button_rotate              { background-color: #7c3aed; color: #fff; border: none; }
#button_rotate:hover        { background-color: #6d28d9; }

#graph_stop_button          { background-color: #d97706; color: #fff; border: none; font-weight: bold; }
#graph_stop_button:hover    { background-color: #b45309; }

#button_fr_constant         { background-color: #db2777; color: #fff; border: none; font-weight: bold; }
#button_fr_constant:hover   { background-color: #be185d; }

#button_cal_constant        { background-color: #ea580c; color: #fff; border: none; font-weight: bold; }
#button_cal_constant:hover  { background-color: #c2410c; }

#save_button                { background-color: #0d9488; color: #fff; border: none; min-height: 32px; }
#save_button:hover          { background-color: #0f766e; }

/* ── LCD Number ─────────────────────────────────────────────── */
QLCDNumber {
    background-color: #1f2937;
    color: #4ade80;
    border: 1px solid #d1d5db;
    border-radius: 4px;
}

/* ── MenuBar / Menu ─────────────────────────────────────────── */
QMenuBar              { background-color: #ffffff; color: #1f2937; border-bottom: 1px solid #e5e7eb; }
QMenuBar::item:selected { background-color: #dbeafe; }
QMenu                 { background-color: #ffffff; color: #1f2937; border: 1px solid #d1d5db; }
QMenu::item:selected  { background-color: #dbeafe; }

/* ── Status bar ─────────────────────────────────────────────── */
QStatusBar { background-color: #f9fafb; color: #6b7280; font-size: 11px; border-top: 1px solid #e5e7eb; }

/* ── Tab widget ─────────────────────────────────────────────── */
QTabWidget::pane  { border: 1px solid #d1d5db; background-color: #ffffff; }
QTabBar::tab      { background-color: #f3f4f6; color: #6b7280; padding: 6px 18px; border-bottom: 2px solid transparent; }
QTabBar::tab:selected { color: #2563eb; border-bottom: 2px solid #2563eb; background-color: #ffffff; }
QTabBar::tab:hover { color: #1f2937; background-color: #e5e7eb; }

/* ── Scrollbars ─────────────────────────────────────────────── */
QScrollBar:vertical   { background: #f3f4f6; width: 8px; border-radius: 4px; }
QScrollBar::handle:vertical { background: #9ca3af; border-radius: 4px; min-height: 20px; }
QScrollBar::handle:vertical:hover { background: #6b7280; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
"""
