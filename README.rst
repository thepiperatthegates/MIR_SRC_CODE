=============================================================================
``Source code for MIR`` 
=============================================================================


Developed by Hijazi Hakkim for his Praxisprojekt and Bachelor-Thesis at Forschungszentrum Jülich in 2024/2025.

Contact info :


i) ``hijaziaziz@gmail.com``

ii) ``h.bin.mohd.aziz@fz-juelich.de``

=====================================================
Running the script 
=====================================================

I) **Windows 10/11**

Follow these steps to execute the Powershell ``.ps1`` script.

Step 1: Open PowerShell as Administrator
----------------------------------------

Before running the script, the host process must have elevated privileges.

1. Press the **Windows Key**.
2. Type ``PowerShell``.
3. Right-click **Windows PowerShell** (or Terminal) and select **Run as Administrator**.


Step 2: Navigate to the Script Directory
----------------------------------------

Use the ``cd`` command to point to your folder. 
*Note: Of course, this depends on where you actually place the parent folder*
For example;

.. code-block:: powershell

   cd C:\Users\MIR_SRC_CODE


Step 3: Executing the Script
--------------------------------------
Firstly, you must ensure ``Get-ExecutionPolicy`` is not set to ``Restricted`` in Powershell. 

If it does, to run the script immediately without changing global system security  
permanently, use the ``-ExecutionPolicy Bypass`` flag.

.. code-block:: powershell

   Set-ExecutionPolicy Bypass -Scope Process -Force

or 

.. code-block:: powershell

   Set-ExecutionPolicy AllSigned

This does not indicate any malicious intent on your system, do not be alarmed!

Run the script now.

.. code-block:: powershell

   .\run.ps1


II) **Linux/macOS/Unix**


Follow these steps to prepare and execute the bash script ``.sh`` script on Linux or macOS.

Step 1: Open the Terminal and Navigate to the Script Directory
---------------------------------------------------------------------------

Use the ``cd`` command to move to the folder containing your script.
*Note: Of course, this depends on where you actually place the parent folder*
For example;

.. code-block:: bash

   cd ~/MIR_SRC_CODE

Step 2: Run the Script with Root (Admin) Privileges
---------------------------------------------------

You have two options to run the script, either directly use the bash command ``bash`` to execute the script, such as:

.. code-block:: bash

    bash run_MIR.sh

or make it executable in the first place by typing the command ``chmod``:

.. code-block:: bash

    chmod +x run_MIR.sh

Afterwards, one could run the script directly by said command:

.. code-block:: bash

    ./run_MIR.sh