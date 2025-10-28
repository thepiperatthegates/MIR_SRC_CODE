#!/usr/bin/env bash

# ---------------------------------------
# Run a Python script on Windows (Git Bash / WSL)
# ---------------------------------------

# Path to your Python script
SCRIPT_PATH="C:\Users\mit\Documents\MIR_SRC_CODE\GUI_SRC_CODE\src\main.py"

# Optional: path to Python (use python3 or python depending on your setup)
PYTHON_CMD="python"



cat <<'EOF'
                               HHHHHHHHHHHHHH
                           $HHHHHH$$$$$$$HHHHHHH
                        HHHHHHHH$$H$$$$$::$$HHHHHH$$
                      HHHHHHHHHHHHHHHHH$$::$$$HHHHH$H$
                   HHHHHHHHHHHHH$HHHHHHHHH$$$$$$HHH$$:$$
                 $HHHH$HHHHHHHHH$$HHHHH$$$HH$$H$$H$$::$H$$
               $HHHH$HHHHHH^^     ^HH^    :HH$HH$$$::$HH$$$
              HHHH$$HHHHH^:                :h$$HH$$$$HHH$H$$
             HHHH$$HHHH^::                  ;$$hHHH$HHHHH$ $
            $HHH$:HHHHH::                   $$ H$H$$H$H$$HHHH$
            $HH$:HHHHHH:::                  H:  $$HHHH$$HHHHh $
            HHH$:$HHHHH::                   H$ ..$$HHHHHHH$$$$H
           $HHHH$$$$HHH$$:::...        ...:$$HH$HH$HHHH$HH$$HH$H
          $HHHHH$HHHHH$:.._.'''':     ````_.._..H^HH$$HHHHHH$$ H
          HHHH$$HHHHHH::=<_(@)~>:\    . /~(@)_>= :HHHH$$$$$HHH$$
         $H$$$::$$HHHH:: ''~~~~~ :`     ~~~~~``  ::H$$HHH$$$$HH$
        $H$HHH$$$HHHHH:          ::  '           : HH::$HHHHHHH $
        HH$HHHH$$$HHHH$          :|  :           :HH$H$::$HHHHH$$
         H$$$HHHHHHHHH$         :/|  :\         ::HHHHHHH$$$$HHHH$
        $$HH$HHHHHHHHH$$       :(:.__.:)  .    :::HH$$$HHHH$:::$$$$
       $$HH$HHH$HHHHHH$$:  .:'             `. '  :HHH$$:::$$HH$$HH$$
      $$HHHHHH$$H$HHHH$$:  : . _..-..-.._ . '    :HHHH$$::$$H$H$$H$H
      $HHHH$$::HHHHHHHH$::    ~=--------=~      .$H$HHHHHHHHH$$:HH$
       H$$$::HHHHHHHHHH$$:.     ~------~       .$HHHHH$$HHHH$$::H$
     $H$$$HHHHHH$$HHHHHHH$::..              ..$HHHHHHHHHHHH$$:$$$$H
    $H$$HH$$$$HHH$$HHHHHHHH$$::          .:$HHHHHHHHHHHHHH$$$$$HHH$
     H$$$HHHH$$HHHH$HHHHHHHHHHHn.__..__.nHHHH$HHHHH%%%HHHHHH$$$HHH
    $HHDrSHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH$HHHHH%%%%%%%%HHHHHH$HH$
   $HHHHHH$HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH$H$HH%%%%%%%%%%%%%HHHHHHH
    $HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH$$$H%%%%%%%%%%%%%%%%%HHHHHH "

EOF


echo "Starting MiR-GUI.............................."
echo "DO NOT CLOSE THIS WINDOW!"
$PYTHON_CMD "$SCRIPT_PATH"


# Optionally, wait for user input before closing (useful in Git Bash)
read -p "Press Enter to exit..."