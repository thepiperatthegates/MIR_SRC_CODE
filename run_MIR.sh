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
		echo $'\e[32mVirtual environment is not found for python script................\e[0m'
		echo $'\e[32mCreating virtual environment for this system.............\e[0m'
		
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
	echo $'\e[32mVirtual environment found.........\e[0m'
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
# Run application
# ---------------------------------------
echo -e "\e[94m"
cat <<'EOF'
                     %%%%%%%%%%%                                
                %%%%%%%%%%%%%%%                                 
            %%%%%%%%%%%%%%%%%%%                                 
          %%%%%%%%%%%%%%%%%%%%                                  
       #%%%%%%%%%%%%%%%%%%%%%                                   
      %%%%%%%%%%%%%%%%%%%%%%                     %%%            
    %%%%%%%%%%%%%%%%%%%%%%%%                    %%%%%%          
   %%%%%%%%%%%%%%%%%%%%%%%%                     %%%%%%%         
  %%%%%%%%%%%%%%%%%%%%%%%%                     %%%%%%%%%        
 %%%%%%%%%%%%%%%%%%%%%%%%%                    %%%%%%%%%%%       
%%%%%%%%%%%%%%%%%%%%%%%%%                    %%%%%%%%%%%%%      
%%%%%%%%%%%%%%%%%%%%%%%%                     %%%%%%%%%%%%%      
%%%%%%%%%%%%%%%%%%%%%%%                     %%%%%%%%%%%%%%%     
%%%%%%%%%%%%%%%%%%%%%%%                    %%%%%%%%%%%%%%%%     
 %%%%%%%%%%%%%%%%%%%%%                    %%%%%%%%%%%%%%%%%%    
  %%%%%%%%%%%%%%%%%%%                     %%%%%%%%%%%%%%%%%%    
   %%%%%%%%%%%%%%%%%                     %%%%%%%%%%%%%%%%%%%    
     %%%%%%%%%%%%                       %%%%%%%%%%%%%%%%%%%%    
                                       %%%%%%%%%%%%%%%%%%%%%    
                                       %%%%%%%%%%%%%%%%%%%%     
                                      %%%%%%%%%%%%%%%%%%%%%     
                                    %%%%%%%%%%%%%%%%%%%%%%      
                                   %%%%%%%%%%%%%%%%%%%%%%%      
                                 %%%%%%%%%%%%%%%%%%%%%%%%       
                               %%%%%%%%%%%%%%%%%%%%%%%%%        
                             %%%%%%%%%%%%%%%%%%%%%%%%%%         
                         %%%%%%%%%%%%%%%%%%%%%%%%%%%%%          
                     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%              
          %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                   
                %%%%%%%%%%%%%%%%%%%%%%%%%%                      
                     %%%%%%%%%%%%%%%#                           

 
EOF
echo -e "\e[0m"


echo "[INFO] Running MiR-GUI..................................."
echo $'\e[1;31mDO NOT CLOSE THIS WINDOW!\e[0m'


python -u $MAIN_SCRIPT_PATH | while IFS= read -r line; do
    echo -e "\e[33m$line\e[0m"
done


# Wait for user input before closing 
read -p $'\e[1;31mPress Enter to exit...\e[0m'