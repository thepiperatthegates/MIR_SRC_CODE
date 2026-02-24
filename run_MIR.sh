
#!/bin/bash
#!/usr/bin/env bash
#set exception if any command fails 

#---------------------------------------
# Configuration
# ---------------------------------------

#Absolute path of where the script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"  
PROJECT_ROOT="$SCRIPT_DIR/GUI_SRC_CODE"

# ---------------------------------------
# Select Python executable per OS
# ---------------------------------------

case "$(uname)" in
  Linux*)
    PYTHON_CMD="python3"
    ;;
  Darwin*|CYGWIN*|MINGW*)
    PYTHON_CMD="python"
    ;;
esac

		
	
REQUIRED_PYTHON="3.12.12"
REQ_FILES="$PROJECT_ROOT/requirements.txt"
MAIN_SCRIPT_PATH="$PROJECT_ROOT/src/main.py"

#Local micromamba executable path
MAMBA_EXE="$SCRIPT_DIR/bin/micromamba"
ENV_DIR="$PROJECT_ROOT/.conda_env"


# OS/Arch Detection for Official URL
OS_NAME="$(uname -s | tr '[:upper:]' '[:lower:]')"
ARCH_NAME="$(uname -m)"

# --------------- WINDOWS (Cygwin or MSYS/Git Bash) ---------------------
if [[ "$OS_NAME" == *"cygwin"* || "$OS_NAME" == *"mingw"* || "$OS_NAME" == *"msys"* ]]; then
    ENV_PYTHON="${ENV_DIR}/python.exe"
    PLATFORM="win-64"
    MAMBA_EXE="${MAMBA_EXE}.exe" # Windows requires .exe extension

# --------------- MACOS (Darwin) ---------------------
elif [[ "$OS_NAME" == "darwin" ]]; then
    ENV_PYTHON="${ENV_DIR}/bin/python"
    
    # Check for Apple Silicon vs Intel
    if [[ "$ARCH_NAME" == "arm64" ]]; then
        PLATFORM="osx-arm64"
    else
        PLATFORM="osx-64"
    fi

# --------------------- Linux ---------------------
else
    ENV_PYTHON="${ENV_DIR}/bin/python"
    
    case "$ARCH_NAME" in
        x86_64)
            PLATFORM="linux-64"
            ;;
        aarch64|arm64)
            PLATFORM="linux-aarch64"
            ;;
        ppc64le)
            PLATFORM="linux-ppc64le"
            ;;
        *)
            echo -e "\e[33mWarning: Unknown architecture $ARCH_NAME. Defaulting to linux-64.\e[0m"
            PLATFORM="linux-64"
            ;;
    esac
fi



#-------------------------------------------------
# CHECK IF MAMBA EXE EXIST 
# IF NOT, INSTALL IT FROM WEB USING CURL COMMAND
# ------------------------------------------------

#IF DIR DOES NOT EXISTS
if [[ ! -f "$MAMBA_EXE" ]]; then 
		echo $'\e[32mMamba environment is not found for the python script................\e[0m'
		echo $'\e[1;32mFetching micromamba bootstrapper for ${OS_NAME}-${ARCH_NAME}.............\e[0m'
		

    #Download link using curl
    curl -L https://micro.mamba.pm/api/micromamba/${PLATFORM}/latest | tar -xvj bin/micromamba

    #make it executable (only on linux and macos)
    chmod +x "$MAMBA_EXE"
		
else
	#Miniconda is available 
	echo $'\e[32mMiniconda is available to use.........\e[0m'
fi
		

# ------------------------------------------------------------------------------------
# Creating python (ESPECIALLY FOR PYTHON 3.12!!!!) environment if no conda virtual env
# ----------------------------------------------------------------------------------
if [[ ! -f "$ENV_PYTHON" ]]; then
  echo -e "\e[32mCreating Python 3.12 environment from conda...\e[0m"

  "$MAMBA_EXE" create -y -p "$ENV_DIR" -c conda-forge python=3.12 pip matplotlib numpy pandas pyqt

  # 2. Use pip ONLY for the remaining items in your requirements file
    echo -e "\e[32mInstalling remaining dependencies...\e[0m"
    "$ENV_PYTHON" -m pip install -r "$REQ_FILES"
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


"$ENV_PYTHON" -u "$MAIN_SCRIPT_PATH"


# Wait for user input before closing 
read -p $'\e[1;31mPress Enter to exit...\e[0m'