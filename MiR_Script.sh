#!/usr/bin/env bash
#set exception if any command fails 

#---------------------------------------
# Configuration
# ---------------------------------------

#FIND THE OS USED IN THE SCRIPT
case "$(uname)" in
	CYGWIN*|MINGW*)
		PROJECT_ROOT="C:/Users/Hijazi/Documents/GitHub/MIR_SRC_CODE/GUI_SRC_CODE"
		;;
	Darwin*)
		PROJECT_ROOT="/Users/mac/Documents/GitHub/MIR_SRC_CODE/GUI_SRC_CODE"
		;;
	Linux*)
		echo "to be worked on"
		;;
esac 
		
	

VENV_DIR="$PROJECT_ROOT/.venv"
REQUIRED_PYTHON="3.12.12"
REQ_FILES="$PROJECT_ROOT/requirements.txt"
MAIN_SCRIPT_PATH="$PROJECT_ROOT/src/main.py"




#---------------------------------------
# CHECK IF ENV EXISTS
# ---------------------------------------

#IF DIR DOES NOT EXISTS
if [[ ! -d "$VENV_DIR" ]]; then 
		echo "Virtual environment is not found for python script................"
		echo "Creating virtual environment for this system............."
		
		python -m venv "$VENV_DIR"
		
		#ACTIVATE THE VIRTUAL ENVIRONMENT 

    case "$(uname)" in
      CYGWIN*|MINGW*)
		    source "$VENV_DIR/Scripts/activate"
        ;;
      Darwin*|Linux*)
		    source "$VENV_DIR/bin/activate"
        ;;
    esac 
		
    python -m pip install --upgrade pip

		#INSTALL REQUIRED PACKAGES
		pip install -r "$REQ_FILES"

    echo "*" > .venv/.gitignore
		
else
	#ACTIVATE THE VIRTUAL ENVIRONMENT 
	echo "Virtual environment found........."
    case "$(uname)" in
      CYGWIN*|MINGW*)
        source "$VENV_DIR/Scripts/activate"
        ;;
      Darwin*|Linux*)
        source "$VENV_DIR/bin/activate"
        ;;
    esac 
fi
		

# ---------------------------------------
# EXECUTE THE PYTHON SCRIPT
# ---------------------------------------
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
echo "[INFO] Running MiR-GUI..................................."
echo "DO NOT CLOSE THIS WINDOW!"
python $MAIN_SCRIPT_PATH


# Optionally, wait for user input before closing (useful in Git Bash)
read -p "Press Enter to exit..."