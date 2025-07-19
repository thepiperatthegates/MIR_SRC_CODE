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

Code were built with Python 3.13.3 in mind (click `here <https://www.python.org/downloads/release/python-3133/>`_), use other version of Python with own risk!



How to run?
------------

Run the main file ``main.py`` with following command line in terminal;

.. code-block:: sh

    python main.py

or if using macOs

.. code-block:: sh

    python3 main.py


**Important note!**: 

The connection between firmware and software depended on the USB port name! Different PC used will use expectedly different port name. To check the port name used by USB-OTG-FS, check;

i) **Windows 10/11**

Select Start, search `Device Manager`, find device port name under `USB COM-Port`


ii) **MacOs or Linux**

Go to terminal and type 

.. code-block:: sh 

    ls /dev/tty.*

It will show bunch of list for USB such as 

.. code-block:: sh 

    /dev/tty.Bluetooth-Incoming-Port	/dev/tty.debug-console      /dev/tty.usbmodem3776345D32331


Pick the ``usbmodem`` as the COM port name. 

Go to ``sockets_files.py`` and change the COM Port name from the global variable ``port_name``.


File location
------------

i) **main.py **

Main files that handles the GUI

ii) **sockets_files.py**

Subfiles that handles connection and connection port between firmware and hardware

iii) **packet_transmission.py**

Subfiles to store functions for use between subfiles


iv) **window_show.py**

Subfiles that handles the extra windows that pops out when data acquisition is commenced, and also another window with subprocess to *calculate rheological properties*


