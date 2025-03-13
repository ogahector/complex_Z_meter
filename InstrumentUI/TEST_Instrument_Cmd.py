import time
import random


class FakeSerial:
    """Fake serial port class to simulate STM32 responses."""

    def __init__(self, port="COM_TEST", baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.buffer = []  # Simulated response buffer
        self.in_waiting = 0  # Simulated buffer size
        self.response_iterator = None  # To iterate over the response list

    def write(self, data):
        """Simulate sending a command to the device."""
        self.buffer = data  # Store command as if it's been sent
        self.in_waiting = len(data)  # Fake data available for read

    def read(self, size=1):
        """Simulate reading the next response from the device."""
        if self.response_iterator is None:
            return b""  # If no responses are set up, return nothing

        # Get the next response from the iterator
        try:
            response = next(self.response_iterator)
            self.in_waiting = len(response)
            return response
        except StopIteration:
            return b""  # No more data to read

    def readline(self):
        """Simulate reading a line of response."""
        return self.read(self.in_waiting)

    def close(self):
        """Simulate closing the serial port."""
        self.is_open = False

    def open(self):
        """Simulate opening the serial port."""
        self.is_open = True

    def flush(self):
        """Simulate flushing the serial buffer."""
        self.buffer = []

    def inWaiting(self):
        """Return number of bytes waiting in buffer (fake buffer)."""
        return self.in_waiting


# - User-defined Command Table
command_table = \
    {
        # ----------------------------------------------------------------
        # System & Testing
        # ----------------------------------------------------------------
        "get_version": 0x0000,
        "get_id": 0x0001,  # STM32 unique device id
        "get_clk_divpw": 0x0002,
        "check_status_pos12": 0x0013,
        "check_status_neg12": 0x0014,
        "check_status_3v3": 0x0015,

        # ----------------------------------------------------------------
        # STM32 Peripheral
        # ----------------------------------------------------------------
        "start_readout_meas": 0x2200,
        "stop_readout_meas": 0x2201,
        "get_phasors": 0x2202,
        "start_sc_calib": 0x2203,
        "start_oc_calib": 0x2204,
        "stop_sc_calib": 0x2205,
        "stop_oc_calib": 0x2206,
        "rref_get_val": 0x2300,
        "rref_set_val": 0x2301,

        # - DAC & ADC
        "dac_set_val": 0x1103,
        "dac_get_val": 0x0103,
        "adc_get_val": 0x0104,
        "curr_get_val": 0x0105,

        # - SPI
        "spi_transfer": 0x1107,
        "spi_init": 0x0107,
        "spi_deinit": 0x0108,

        # ----------------------------------------------------------------
        # - System Commands
        "quit": 0x0F00,  # System cmd: Quit the command interface
        "list_cmd": 0x0F01,  # System cmd: List all commands
        "list_serial": 0x0F10,  # System cmd: List serial port
        "close_serial": 0x0F11,  # System cmd: Close current serial port
        "open_serial": 0x1F12,  # System cmd: Open serial port COM [par1]
    }

command_list = list(command_table.keys())


class TESTInstrumentCmd(object):
    """Mock class to simulate STM32 serial communication."""

    def __init__(self):
        self.serial_connected = True
        self.console_s = None  # This should be connected to UI
        self.serial_obj = FakeSerial()  # Use the fake serial class
        self.serial_port = None
        self.response_iterator = None  # Iterator for the phasor data

    def is_connected(self):
        return self.serial_connected

    def open_serial(self, port="COM6"):
        """Simulate opening a serial port."""
        self.serial_connected = True
        self.serial_obj.open()
        if self.console_s:
            self.console_s.emit(f"TEST: Connected to {port}\n")

    def close_serial(self):
        """Simulate closing a serial port."""
        self.serial_connected = False
        self.serial_obj.close()
        if self.console_s:
            self.console_s.emit("TEST: Serial port closed.\n")

    def execute_cmd(self, command, param=None):
        """Simulate command execution and return test values."""
        time.sleep(0.1)  # Simulate processing delay
        test_responses = {
            "get_version": 1,
            "get_id": 12345678,
            "get_clk_divpw": 2,
            "check_status_pos12": 1,
            "check_status_neg12": 1,
            "check_status_3v3": 1,
            "adc_get_val": random.randint(0, 4095),  # Simulating ADC values
            "curr_get_val": round(random.uniform(0.1, 2.0), 3),  # Simulating current values
            "dac_get_val": 512,
            "rref_get_val": 1000,
            "start_sc_calib": [
                (10, 1.2, 80),  # Frequency: 10Hz, Magnitude: 1.2, Phase: 80 degrees
                (100, 0.9, 75),  # Frequency: 20Hz, Magnitude: 0.9, Phase: 75 degrees
                (1000, 0.7, 60),  # Frequency: 30Hz, Magnitude: 0.7, Phase: 60 degrees
                (10000, 0.5, 45)   # Frequency: 40Hz, Magnitude: 0.5, Phase: 45 degrees
            ],
            "start_oc_calib": [
                (10, 0.6, -50),  # Frequency: 50Hz, Magnitude: 0.6, Phase: -50 degrees
                (100, 0.8, -60),  # Frequency: 60Hz, Magnitude: 0.8, Phase: -60 degrees
                (1000, 1.0, -70),  # Frequency: 70Hz, Magnitude: 1.0, Phase: -70 degrees
                (10000, 1.2, -80)   # Frequency: 80Hz, Magnitude: 1.2, Phase: -80 degrees
            ],
            "readout_meas": [
                (10, 2.5, 0),    # Frequency: 100Hz, Magnitude: 2.5, Phase: 0 degrees
                (100, 2.2, 5),    # Frequency: 200Hz, Magnitude: 2.2, Phase: 5 degrees
                (1000, 1.8, -5),   # Frequency: 300Hz, Magnitude: 1.8, Phase: -5 degrees
                (10000, 1.5, -10)   # Frequency: 400Hz, Magnitude: 1.5, Phase: -10 degrees
            ],
            "stop_sc_calib": 1,
            "stop_oc_calib": 1,
            "stop_readout_meas": 1
        }

        # Handle writing commands to serial (simulating MCU response)
        if command in test_responses:
            response = test_responses.get(command, f"TEST: Unknown command {command}")

            # Handle the case for 'readout_meas' command
            if command == "readout_meas":
                if self.response_iterator is None:
                    self.response_iterator = iter(response)  # Initialize iterator for first call
                phasor = next(self.response_iterator, None)  # Get next phasor
                if phasor is not None:
                    if self.console_s:
                        self.console_s.emit(f"S: Readout: {phasor}\n")
                    else:
                        print(f"S: Readout: {phasor}\n")
                return [phasor]  # Return single phasor

            # Handle the case for 'start_sc_calib' and 'start_oc_calib' commands
            elif command in ["start_sc_calib", "start_oc_calib"]:
                if self.response_iterator is None:
                    self.response_iterator = iter(response)  # Initialize iterator for first call
                phasor = next(self.response_iterator, None)  # Get next phasor
                if phasor is not None:
                    if self.console_s:
                        self.console_s.emit(f"S: {command} {phasor}\n")
                    else:
                        print(f"S: {command} {phasor}\n")
                return [phasor]  # Return single phasor

            # Handle the case for 'stop_sc_calib' command (reset the iterator)
            elif command in ["stop_sc_calib", "stop_oc_calib"]:
                print("Stopping calibration, resetting iterator.")
                self.response_iterator = None  # Reset the iterator when stop command is received
                return "S: Calibration Stopped"

            # Handle the case for 'stop_sc_calib' command (reset the iterator)
            elif command == "stop_readout_meas":
                print("Stopping measurement, resetting iterator.")
                self.response_iterator = None  # Reset the iterator when stop command is received
                return "S: Measurement Stopped"

            return response
        else:
            return "TEST: Invalid command"

    def list_serial(self, show=False):
        """Simulate listing available serial ports."""
        return ["TESTPort1", "TESTPort2"], ["COM3", "COM4"]

    def clear(self):
        """Simulate clearing the serial buffer."""
        self.serial_obj.flush()

    def print(self, message):
        if self.console_s is None:
            print(message, end='')
        else:
            self.console_s.emit(message)
