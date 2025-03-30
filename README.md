# actuonix_lac
Python controller for Actuonix Linear Actuator Control Board 
Modified from https://github.com/DamnedFacts/actuonix-lac

# Tested on
Actuonix L16-P Linear Actuators with 5-wire connectors.
    https://www.actuonix.com/l16-p
Actuonix Linear Actuator Control Board.
    https://www.actuonix.com/lac

# Requirements
1. The PyUSB package should be installed.
2. Make sure the backend is installed:
    "conda install -c conda-forge libusb"     # For Conda users
    OR "pip install libusb"                   # For Pip users
3. Double-check the VendorID and DeviceID:
    Windows "Device Manager" -> "Custom USB Devices" -> "WinUSB Devices"
    -> "Hardware Ids" -> "USB\VID_04D8&PID_FC5F"
    VID = 0x04d8  # Vendor ID
    PID = 0xfc5f  # Product ID
   
