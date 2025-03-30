from ctypes import WinDLL
import sys

# Control Command Constants (Expanded)
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
# Actuonix Vendor/Product ID
# Windows "Device Manager" -> "Custom USB Devices" -> "WinUSB Devices"
# -> "Hardware Ids" -> "USB\VID_04D8&PID_FC5F"
DEFAULT_VID_PID = b"vid_04d8&pid_fc5f"
ALTERNATIVE_IDS = [
    b"vid_04d8&pid_000c",  # Secondary
    b"vid_04d8&pid_01f0",
    b"vid_04d8&pid_01f1",
    b"vid_04d8&pid_01f2",
    b"vid_04d8&pid_01f3",
    b"vid_04d8&pid_01f4",
    b"vid_04d8&pid_01f5",
    b"vid_04d8&pid_01f6"
]
DEFAULT_ENDPOINT_OUT = b"\\MCHP_EP1"  # OUT endpoint
DEFAULT_ENDPOINT_IN = b"\\MCHP_EP2"  # IN endpoint


class ActuonixLAC:
    def __init__(self, vid_pid=DEFAULT_VID_PID, instance=0):
        self.vid_pid = vid_pid
        self.instance = instance
        self.dll = WinDLL(r"C:\Program Files (x86)\Actuonix LAC Configuration Utility\mpusbapi.dll")
        self._setup_function_prototypes()
        self.INHandle = None
        self.OUTHandle = None

    def _setup_function_prototypes(self):
        """Define DLL function signatures"""
        self.dll.MPUSBOpen.argtypes = [c_ulong, c_char_p, c_char_p, c_ulong, c_ulong]
        self.dll.MPUSBOpen.restype = c_void_p

        self.dll.MPUSBWrite.argtypes = [c_void_p, c_void_p, c_ulong, POINTER(c_ulong), c_ulong]
        self.dll.MPUSBWrite.restype = c_ulong

        self.dll.MPUSBRead.argtypes = [c_void_p, c_void_p, c_ulong, POINTER(c_ulong), c_ulong]
        self.dll.MPUSBRead.restype = c_ulong

        self.dll.MPUSBClose.argtypes = [c_void_p]
        self.dll.MPUSBClose.restype = c_ulong

    def connect(self):
        """Establish connection to LAC device"""
        self.OUTHandle = self.dll.MPUSBOpen(
            self.instance, self.vid_pid, DEFAULT_ENDPOINT_OUT, 0, 0
        )
        self.INHandle = self.dll.MPUSBOpen(
            self.instance, self.vid_pid, DEFAULT_ENDPOINT_IN, 1, 0
        )
        if not self.OUTHandle or not self.INHandle:
            raise ConnectionError("Failed to open device handles")

    def close(self):
        """Close active connections"""
        if self.OUTHandle:
            self.dll.MPUSBClose(self.OUTHandle)
        if self.INHandle:
            self.dll.MPUSBClose(self.INHandle)
        self.OUTHandle = None
        self.INHandle = None

    def _write_command(self, control, data_low=0, data_high=0):
        """Internal method to send commands"""
        if not self.OUTHandle:
            raise ConnectionError("Device not connected")
        buffer = create_string_buffer(bytes([control, data_low, data_high]))
        actual_length = c_ulong(0)
        if self.dll.MPUSBWrite(self.OUTHandle, buffer, 3, byref(actual_length), 1000) != 1:
            raise IOError(f"Write failed for command 0x{control:02X}")
        return actual_length.value

    def _read_response(self):
        """Internal method to read responses"""
        if not self.INHandle:
            raise ConnectionError("Device not connected")
        buffer = create_string_buffer(3)
        actual_length = c_ulong(0)
        if self.dll.MPUSBRead(self.INHandle, buffer, 3, byref(actual_length), 1000) != 1:
            raise IOError("Read operation failed")
        return bytes(buffer)

    def send_command(self, control, data=0):
        """Generic command sender with validation"""
        if not 0 <= data <= 0xFFFF:
            raise ValueError("Data value out of range (0-65535)")

        data_low = data & 0xFF
        data_high = (data >> 8) & 0xFF
        self._write_command(control, data_low, data_high)
        response = self._read_response()

        if response[0] != control:
            raise ValueError(f"Response mismatch. Sent: 0x{control:02X}, Received: 0x{response[0]:02X}")

        # Process response data
        response_data = response[1] | (response[2] << 8)
        return response_data

    # Device Configuration Commands
    def set_accuracy(self, value):
        """Set accuracy threshold (0-1023)"""
        if not 0 <= value <= 1023:
            raise ValueError("Accuracy must be 0-1023")
        return self.send_command(SET_ACCURACY, value)

    def set_limits(self, retract, extend):
        """Set both movement limits (0-1023)"""
        self.set_retract_limit(retract)
        self.set_extend_limit(extend)

    def set_retract_limit(self, value):
        """Set retract limit (0-1023)"""
        if not 0 <= value <= 1023:
            raise ValueError("Limit must be 0-1023")
        return self.send_command(SET_RETRACT_LIMIT, value)

    def set_extend_limit(self, value):
        """Set extend limit (0-1023)"""
        if not 0 <= value <= 1023:
            raise ValueError("Limit must be 0-1023")
        return self.send_command(SET_EXTEND_LIMIT, value)

    def set_position(self, position, stroke=None):
        """Set target position (0-1023) with optional stroke conversion"""
        if stroke:
            position = int((position / stroke) * 1023)
        if not 0 <= position <= 1023:
            raise ValueError("Position must be 0-1023")
        return self.send_command(SET_POSITION, position)

    def set_speed(self, speed):
        """Set movement speed (0-1023)"""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be 0-1023")
        return self.send_command(SET_SPEED, speed)

    # Advanced Configuration
    def set_pid(self, kp, kd):
        """Set proportional and derivative gains (0-1023)"""
        self.set_kp(kp)
        self.set_kd(kd)

    def set_kp(self, value):
        """Set proportional gain (0-1023)"""
        if not 0 <= value <= 1023:
            raise ValueError("Kp must be 0-1023")
        return self.send_command(SET_PROPORTIONAL_GAIN, value)

    def set_kd(self, value):
        """Set derivative gain (0-1023)"""
        if not 0 <= value <= 1023:
            raise ValueError("Kd must be 0-1023")
        return self.send_command(SET_DERIVATIVE_GAIN, value)

    # System Commands
    def save_config(self):
        """Save current configuration to EEPROM"""
        return self.send_command(DISABLE_MANUAL, 0)

    def factory_reset(self):
        """Reset to factory defaults"""
        return self.send_command(RESET, 0)

    # Monitoring
    def get_feedback(self):
        """Read current actuator position"""
        return self.send_command(GET_FEEDBACK, 0)


if __name__ == "__main__":
    lac = ActuonixLAC()
    try:
        lac.connect()

        # Example usage
        print("Current position:", lac.get_feedback())
        # lac.set_position(512)  # Move to midpoint
        # lac.set_speed(800)  # Set movement speed
        # lac.set_pid(500, 300)  # Tune PID values
        # lac.save_config()  # Save settings

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        lac.close()