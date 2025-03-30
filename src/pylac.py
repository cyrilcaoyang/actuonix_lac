import sys, time, struct
import usb.core
import usb.backend.libusb1

# Modified from
# https://github.com/DamnedFacts/actuonix-lac

# Make sure backend is installed:
# "conda install -c conda-forge libusb"     # For Conda users
# OR "pip install libusb"                   # For Pip users

# Specify the libusb backend
backend = usb.backend.libusb1.get_backend()


class ActuonixLAC:
    # Control Command Constants
    SET_ACCURACY = 0x01
    SET_RETRACT_LIMIT = 0x02
    SET_EXTEND_LIMIT = 0x03
    SET_MOVEMENT_THRESHOLD = 0x04
    SET_STALL_TIME = 0x05
    SET_PWM_THRESHOLD = 0x06
    SET_DERIVATIVE_THRESHOLD = 0x07
    SET_DERIVATIVE_MAXIMUM = 0x08
    SET_DERIVATIVE_MINIMUM = 0x09
    SET_PWM_MAXIMUM = 0x0A
    SET_PWM_MINIMUM = 0x0B
    SET_PROPORTIONAL_GAIN = 0x0C
    SET_DERIVATIVE_GAIN = 0x0D
    SET_AVERAGE_RC = 0x0E
    SET_AVERAGE_ADC = 0x0F
    GET_FEEDBACK = 0x10
    SET_POSITION = 0x20
    SET_SPEED = 0x21
    DISABLE_MANUAL = 0x30
    RESET = 0xFF

    # Default Configuration (Update with your device's details)
    # Windows "Device Manager" -> "Custom USB Devices" -> "WinUSB Devices"
    # -> "Hardware Ids" -> "USB\VID_04D8&PID_FC5F"
    VID = 0x04d8  # Vendor ID
    PID = 0xfc5f  # Product ID

    def __init__(self):
        self.device = usb.core.find(
            idVendor=self.VID,
            idProduct=self.PID,
            backend=usb.backend.libusb1.get_backend()
        )
        if self.device is None:
                raise Exception("LAC is not found. Make sure LAC is connected with the right VID/PID.")
        else:
            for cfg in self.device:
                print(f"Configuration: {cfg.bConfigurationValue}")
                for intf in cfg:
                    print(f"\tInterface: {intf.bInterfaceNumber}")
                    print(f"\tConfirm 0x01 (OUT) and 0x81 (IN) match your device:")
                    for ep in intf:
                        print(f"\tEndpoint: 0x{ep.bEndpointAddress:02X}")
        self.device.set_configuration()

        # Claim interface 0
        usb.util.claim_interface(self.device, 0)

    # Take data and send it to LAC
    def send_data(self, function, value=0):
        # Validat Value
        if value < 0 or value > 1023:
            raise ValueError("Value is invalid. Must be 2-byte integer in rage [0, 1023]")

        # Send data
        data = struct.pack(
            b'BBB', function, value & 0xFF,
            (value & 0xFF00) >> 8
        )  # Low byte masked in, high byte masked and moved down
        bytes_written = self.device.write(1, data, timeout=100)
        if bytes_written != len(data):
            raise usb.core.USBError("USB write failed.")
        time.sleep(.1)

        # Read response
        response = self.device.read(0x81, 3, timeout=100)  # there's three bytes to a packet
        # High byte moved left, then tack on the low byte
        return (response[2] << 8) + response[1]

    def set_accuracy(self, value=4):
        # Accuracy tolerance [0, 1023]
        # Would result in "shaking" if set too low
        # value/1024 * stroke gives accuracy in distance,
        # where stroke is max extension length (all values in mm). Round to nearest integer
        self.send_data(self.SET_ACCURACY, value)

    def set_retract_limit(self, value=0):
        # The maximum retraction [0, 1023]
        # Going to 0 will hit end stop, please avoid
        self.send_data(self.SET_RETRACT_LIMIT, value)

    def set_extend_limit(self, value=1023):
        # The maximum extension [0, 1023]
        # Going to 1023 will hit end stop, please avoid
        self.send_data(self.SET_EXTEND_LIMIT, value)

    def set_movement_threshold(self, value=3):
        # Minimum speed before actuator is considered stalling
        self.send_data(self.SET_MOVEMENT_THRESHOLD, value)

    def set_stall_time(self, value=10000):
        # Timeout (ms) before actuator shuts off after stalling
        self.send_data(self.SET_STALL_TIME, value)

    def set_pwm_threshold(self, value=80):
        # When feedback-set>this, set speed to maximum
        self.send_data(self.SET_PWM_THRESHOLD, value)

    def set_derivative_threshold(self, value=3):
        # Compared to measured speed to determine
        # PWM increase (prevents stalls). Normally equal to movement threshold
        self.send_data(self.SET_DERIVATIVE_THRESHOLD, value)

    def set_max_derivative(self, value=1023):
        # Maximum value the D term can contribute to control speed
        self.send_data(self.SET_MAX_DERIVATIVE, value)

    def set_min_derivative(self, value=0):
        # Minimum value the D term can contribute to control speed
        self.send_data(self.SET_MIN_DERIVATIVE, value)

    def set_max_pwm_value(self, value=1023):
        # Speed the actuator runs at when outside the pwm threshold 1023 enables top speed
        # though actuator may try to move faster to avoid stalling
        self.send_data(self.SET_MAX_PWM_VALUE, value)

    def set_min_pwm_value(self, value=80):
        # Minimum PWM value applied by PD
        self.send_data(self.SET_MIN_PWM_VALUE, value)

    def set_proportional_gain(self, value=1):
        # Higher value = faster approach to target, but also more overshoot
        self.send_data(self.SET_PROPORTIONAL_GAIN, value)

    def set_derivative_gain(self, value=10):
        # Rate at which differential portion of controller increases while stalling.
        # Not a /real/ differential term, but similar effect.
        # When stalling, derivtive term is incremented to attempt escape
        self.send_data(self.SET_DERIVATIVE_GAIN, value)

    def set_average_rc(self, value=4):
        # Number of samples used in filtering the RC input signal before the actuator moves.
        # High value = more stability, but lower response time. value * 20ms = delay time.
        # This does NOT affect filter feedback delay;
        # Control response to valid input signals is unaffected
        self.send_data(self.SET_AVERAGE_RC, value)

    def set_average_adc(self, value=8):
        # Number of samples used in filtering the feedback and analog input signals, if active.
        # Similar delay effect to set_average_rc, but this DOES affect control response.
        # PD loop values may need to be retuned when adjusting this
        self.send_data(self.SET_AVERAGE_ADC, value)

    def get_feedback(self):
        # Ssend a feedback packet containing its current position.
        # This is read directly from ADC.
        # Will not be equal to the set point if it is yet reached.
        return self.send_data(self.GET_FEEDBACK)

    def set_position(self, value):
        # Set the LAC's position relative to retraction stop [0, 1023].
        # value = int ( (distance * 1023)/stroke ).
        # Note that this will disable RC, I, and V inputs until reboot.
        self.send_data(self.SET_POSITION, value)

    def set_speed(self, value=1023):
        # Set speed of movement [0,1023]
        self.send_data(self.SET_SPEED, value)

    def disable_manual(self):
        # Saves current config to EEPROM and disables all four potentiometers.
        # On reboot, these values will continue being used instead of the potentiometer values.
        # Analog inputs function as normal either way.
        self.send_data(self.DISABLE_MANUAL)

    def reset(self):
        # Enables manual control potentiometers and resets config to factory default
        self.send_data(self.RESET)


if __name__ == "__main__":
    lac = ActuonixLAC()
    try:
        # Read current position first
        print("Current position:", lac.get_feedback())

        # Set movement accuracy and speed
        motor_speed = 800
        motor_accuracy = 8
        lac.set_speed(value=motor_speed)
        lac.set_accuracy(value=motor_accuracy)
        print(f"Speed set to {motor_speed}, accuracy to {(1-motor_accuracy/1024):.2f}.")

        for pos in [8, 512, 1015]:
            lac.set_position(pos)   # Move to start
            print(f"Movint position (0, 1023) to {pos}.")
            time.sleep(5)
            print("Current position:", lac.get_feedback())

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
