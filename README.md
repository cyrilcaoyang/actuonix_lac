# Package name: actuonix_lac
Python controller for Actuonix Linear Actuator Control Board 
Modified from https://github.com/DamnedFacts/actuonix-lac

# Tested on
Actuonix L16-P Linear Actuators with 5-wire connectors.

    https://www.actuonix.com/l16-p
Actuonix Linear Actuator Control (LAC) Board.

    https://www.actuonix.com/lac
Connect the actuator to the LAC and the LAC to your computer using a mini-USB cable.
    
# Requirements
1. The PyUSB package should be installed.
2. Make sure the backend is installed:
   
   for conda:
   
        conda install -c conda-forge pyusb libusb
   for pip:

        pip install libusb
3. Make sure the LAC driver is installed:

        https://www.actuonix.com/assets/images/Actuonix%20LAC%20Configuration%20Utility-24-Setup.zip
4. Double-check the VendorID and DeviceID:

   Windows "Device Manager" -> "Custom USB Devices" -> "WinUSB Devices"

   -> "Hardware Ids" -> "USB\VID_04D8&PID_FC5F"

       VID = 0x04d8  # Vendor ID
       PID = 0xfc5f  # Product ID
