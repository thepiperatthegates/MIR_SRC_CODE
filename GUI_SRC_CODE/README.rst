=============================================================================
``SRC code GUI Python`` 
=============================================================================


This is the source code for GUI Python, intended for usage with MIR.




Needed packages 
------------

Exact list for required libraries is to be found in ``requirements.txt``

For an automatic installation of all packages, copy and run following command in terminal;

.. code-block:: sh

    pip install -r requirements.txt


**Important note!**: 

Code were built with Python 3.13.3 in mind (click `here <https://www.python.org/downloads/release/python-3133/>`_), use other version of Python with your own risk!



How to run?
------------

Make sure your terminal's current working directory is set to the project folder. Example if the folder is in the following PATH;

.. code-block:: sh 

    C:/Users/mac/Documents/YOUR_FILE_HERE

change the directory by using 

.. code-block:: sh 

    
    cd 'C:/Users/mac/Documents/YOUR_FILE_HERE'

Now run the main file ``main.py`` with following command line in terminal;

.. code-block:: sh

    python main.py

or if using macOs

.. code-block:: sh

    python3 main.py


**Important note!**: 

The connection between firmware and software depended on the USB port name! Different PC used will use expectedly different port name. To check the port name used by USB-OTG-FS, check;

i) **Windows 10/11**

Select Start, search `Device Manager`, find device port name under `USB COM-Port`


ii) **MacOs**

Go to terminal and type 

.. code-block:: sh 

    ls /dev/tty.*

It will show bunch of list for USB such as 

.. code-block:: sh 

    /dev/tty.Bluetooth-Incoming-Port	/dev/tty.debug-console      /dev/tty.usbmodem3776345D32331


Pick the ``usbmodem`` as the COM port name. 

Go to ``sockets_files.py`` and change the COM Port name from the global variable ``port_name``.

iii) **Linux**

Go to bash terminal and type 

.. code-block:: sh 

    ls /dev/tty.*

to view all serial connection.

Copy ``/dev/ttyACM0`` as the COM port name. Common problem is that the Linux kernel will not give permission for Python script to access the com port. 

To fix this, simply give the read and write access to the program by typing the following in the terminal;

.. code-block:: sh 

    sudo chmod 666 /dev/ttyACM0

Directory and file types
------------

i) **main.py**

Main files that handles the GUI

ii) **sockets_GUI**

Directory that handles connection and connection port between firmware and hardware

iii) **packet_transmission.py**

File to store functions for use between subfiles

iv) **constant_shear_rate**

Directory containing data visualization, data acquisition, and analysis files for constant shear rate experiments.

