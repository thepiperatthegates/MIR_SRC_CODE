=============================================================================
``SRC code STM32H757XI with STM32H757STM32H757I-EVAL`` 
=============================================================================


How to run?
------------

There is few ways to flash the chips with source code. Below are options that I did during thesis;


Toolchain : 
i) STM32CubeIDE (original toolchain)

ii) VSCode with STM32CubeMX extension, clangd for compiler front-end, arm-none-eabi-gcc from ARM GNU for on-board chip flashing, ST-Link GBD for debugging tool-chain, CMake for cross-compile software development


STM32CubeIDE is recommended as it requires the least amount of step needed to compile, debug and flash the code onto the chips, 
but the IDE is very slow in this case. 


File location
------------

/STM_SRC_CODE/again_try/CM7/Src/main.c is the MAIN source code file, run here!.

For filtering source code, check conv-direct.c and conv-direct.h for headers file.


Libraries
------------

i) **ARM DSP libraries** 

DSP libraries for filtering and controller functions are provided by ARM under the link `here <https://github.com/ARM-software/CMSIS-DSP>`_.

ii) **USB and HAL USB libraries** 

I suggest that you take a look at the main page of STMicroelectronics since version is depended on the update by STM.


Setting up for use with VS-Code (if)
------------

Tutorial to set up the toolchain is provided `here <https://www.youtube.com/watch?v=aWMni01XGeI>`_, and `here <https://marketplace.visualstudio.com/items?itemName=stmicroelectronics.stm32-vscode-extension&ssr=false#overview>`_. 
