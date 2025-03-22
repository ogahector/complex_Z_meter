# Debug Commands
# - STM32 <-> USB VCP <-> PC

# ----------------------------------------------------------------
# Import
# ----------------------------------------------------------------
import serial
import serial.tools.list_ports

import time
import struct
import numpy as np
import ast

# ----------------------------------------------------------------
# User-defined Commands
# ----------------------------------------------------------------
# ! Not secure, research and test only !
# Function:                             0x0??? + 0xXXXX + 0xXXXX
# Function + Parameter 1:               0x1??? + 0x???? + 0xXXXX
# Function + Parameter 1 + Parameter 2: 0x2??? + 0x???? + 0x????

# - User-defined Command Table
command_table = \
{   
    # ----------------------------------------------------------------
    # System & Testing
    # ----------------------------------------------------------------
    "get_version":                      0x0000, 
    "get_id":                           0x0001,     # STM32 unique device id
    "get_clk_divpw":                    0x0002,
    "check_status_pos12":               0x0013,
    "check_status_neg12":               0x0014,
    "check_status_3v3":                 0x0015,

    # ----------------------------------------------------------------
    # STM32 Peripheral
    # ----------------------------------------------------------------
    "readout_meas":                     0x0200,
    "start_rlc_fit":                    0x0202,
    "start_sc_calib":                   0x0203,
    "start_oc_calib":                   0x0204,
    "stop_sc_calib":                    0x0205,
    "stop_oc_calib":                    0x0206,
    "start_ld_calib":                   0x0207,
    "stop_ld_calib":                    0x0208,
    "rref_get_val":                     0x0300,
    "rref_set_val":                     0x1301,

    # - DAC & ADC
    "dac_set_val":                      0x1103, 
    "dac_get_val":                      0x0103, 
    "adc_get_val":                      0x0104,
    "curr_get_val":                     0x0105,

    # - SPI
    "spi_transfer":                     0x1107, 
    "spi_init":                         0x0107, 
    "spi_deinit":                       0x0108,

    # ----------------------------------------------------------------
    # - System Commands
    "quit":                             0x0F00,     # System cmd: Quit the command interface
    "list_cmd":                         0x0F01,     # System cmd: List all commands
    "list_serial":                      0x0F10,     # System cmd: List serial port
    "close_serial":                     0x0F11,     # System cmd: Close current serial port
    "open_serial":                      0x1F12,     # System cmd: Open serial port COM [par1]
}

command_list = []
for (key, value) in command_table.items():
    command_list.append(key)

# - User Command Interface
# noinspection PyBroadException
class DebugCommand(object):
    
    # Assign pyqtSignal directly to those variables instead of connecting
    # - to minimize the module dependencies required
    status_s    = None  # Signal for status indicating
    console_s   = None  # Signal for console print
    msgbox_s    = None  # Signal for msgbox print

    def __init__(self):
        self.serial_obj         = None  # Instance
        self.serial_port        = None  # COM Port
        self.serial_connected   = False # State
        self.serial_ready       = True  # Mutex
        self.serial_timeout     = 5000000

    def set_timeout(self, time_cnt):
        self.serial_timeout = time_cnt

    def is_connected(self):
        return self.serial_connected
  
    # Print information
    def print(self, message):
        if self.console_s is None:
            print(message,  end='')
        else:
            self.console_s.emit(message)
    
    # Status indicator, True: busy, False: idle
    def status(self, state):
        if self.status_s is not None:
            self.status_s.emit(state)
    
    # Error notification
    def error(self, msg):
        if self.msgbox_s is not None:
            self.msgbox_s.emit(msg)
        else:
            print(msg)
            
    def clear(self):
        self.serial_obj.read(self.serial_obj.inWaiting())

    # List serial ports
    def list_serial(self, show=False):
        self.status(True)
        #--------------------------------
        port_list = []
        port_name = []
        
        # List available serial port
        ports = serial.tools.list_ports.comports()
        for port in ports:
            #hwid = port.hwid

            if port.vid==0x0483:
                port_name.append('USB: ' + hex(port.pid)[2:].upper().zfill(4))
                port_list.append(port.device)
                if show:
                    print(port_name[-1] + '(%s)' % port.device)

        #--------------------------------
        self.status(False)
        return port_name, port_list

    # Connect to serial port
    def open_serial(self, port='COM6'):
        self.status(True)
        self.serial_port = port
        self.print('S: Open serial port %s, ' % self.serial_port)
        # Delay for bluetooth SPP
        time.sleep(1)
        try:
            self.serial_obj = serial.Serial(self.serial_port, baudrate = 115200, timeout=30)
            self.serial_obj.set_buffer_size(16*1024*1024)
            self.print('Successful\n')
            self.serial_connected = True
        except Exception as e:
            self.print('Failed\n')
            e_msg = str(e)
            if 'OSError' in e_msg:
                self.error('Timeout !\nMake sure instrument is powered on\n')
            elif 'PermissionError' in e_msg:
                self.error('Serial port is currently occupied by another UI')
            else:
                self.error('Unknown error')
            self.status(False)
            raise SystemError
        else:
            self.status(False)
    
    # Close current serial port
    def close_serial(self):
        self.print('S: Close serial port %s, ' % self.serial_port)
        if self.serial_connected:
            self.serial_obj.close()
            self.print('Successful\n')
            self.serial_obj = None
            self.serial_connected = False
            self.serial_ready = True
        else:
            self.print('No serial port connected\n')    # ? Close serial before opening
    
    # Decode command
    def decode_cmd(self, input_command):
        cmd = input_command.split(' ')
        # Decode function
        try:
            func = command_table[cmd[0]]
        except:
            self.print("S: No such command '%s'\n" % cmd[0])
            return None
        n_para = func>>12
        if n_para==0:
            return func, None, None
        # Decode parameter 1
        try:
            par1 = int(cmd[1], 0)
        except ValueError:
            self.print("S: Invalid parameter '%s', must be a number\n" % cmd[1])
            return None
        except IndexError:
            self.print("S: Command '%s' requires %s parameter(s)\n" % (cmd[0], 'a' if n_para==1 else "two"))
            return None
        if n_para == 1:
            return func, par1, None
        # Decode parameter 2
        try:
            par2 = int(cmd[2], 0)
        except ValueError:
            self.print("S: Invalid parameter '%s', must be a number\n" % cmd[2])
            return None
        except IndexError:
            self.print("S: Command '%s' requires two parameters\n" % cmd[0])
            return None
        return func, par1, par2

        # Execute command
    def execute_cmd(self, input_command):
        import time, ast, struct

        # Decode the command; if invalid, return immediately.
        cmd_decoded = self.decode_cmd(input_command)
        if cmd_decoded is None:
            return None
        else:
            (func, par1, par2) = cmd_decoded

        print(f'Sending command {input_command}: {cmd_decoded}')

        # ------------------------------
        # Handle UI/System Commands (for example, quitting, listing ports, etc.)
        # ------------------------------
        if (func >> 8) & 0x0F == 0x0F:
            if func == 0x0F00:
                if self.console_s is None:
                    cmd = input("Are you sure to quit? y/n: ")
                    if cmd.lower() == 'y':
                        quit(0)
                else:
                    self.print("S: Invalid cmd for UI\n")
            elif func == 0x0F01:
                for index, command in enumerate(command_list):
                    self.print(f"[{index:2d}] " + command + '\n')
            elif func == 0x0F10:
                self.list_serial(show=True)
            elif func == 0x0F11:
                self.close_serial()
            elif func == 0x1F12:
                self.open_serial('COM' + str(par1))
            return None

        # ------------------------------
        # For MCU commands: Ensure the serial port is connected
        # ------------------------------
        if not self.serial_connected:
            self.print('S: Serial port not connected\n')
            self.error('Serial port not connected\n')
            raise SystemError("Serial port not connected")

        # Pack the command into a binary packet (2 bytes each, big-endian)
        cmd_send = struct.pack(">H", func)
        cmd_send += struct.pack(">H", par1 if par1 is not None else 0x00)
        cmd_send += struct.pack(">H", par2 if par2 is not None else 0x00)

        # ------------------------------
        # Enter Critical Region (wait for serial_ready flag)
        # ------------------------------
        while not self.serial_ready:
            pass
        self.serial_ready = False
        self.status(True)
        self.clear()  # Clear serial buffer before sending

        # Send the command to the MCU
        try:
            self.serial_obj.write(cmd_send)
            print(f'Sent command: {cmd_send}')
        except Exception as e:
            self.error("Serial: Write Command Failed")
            self.status(False)
            self.serial_ready = True
            raise SystemError("Write failed") from e

        # ------------------------------
        # Read the Response according to the Protocol:
        # The MCU sends:
        #   [Message Text] + '$' + [Numeric Data (may be sent in batches)] + '#'
        # ------------------------------

        # First, receive the text message (until the '$' delimiter)
        received_message = ''
        timeout_cnt = 0
        while '$' not in received_message:
            try:
                if self.serial_obj.inWaiting() > 0:
                    rdata = self.serial_obj.read(self.serial_obj.inWaiting())
                    rdata = rdata.decode('utf-8')
                    received_message += rdata
                else:
                    time.sleep(0.001)
                    timeout_cnt += 1
                    if timeout_cnt >= self.serial_timeout:
                        self.error('Serial: Timeout waiting for message part')
                        self.status(False)
                        self.serial_ready = True
                        raise TimeoutError("Timeout waiting for message part")
            except Exception as e:
                print(str(e))
                self.error("Serial: Read Message Failed")
                self.status(False)
                self.serial_ready = True
                raise RuntimeError("Error reading message part") from e

        # Separate the message part from the data part
        msg_end_index = received_message.find('$')
        message_part = received_message[:msg_end_index]
        if message_part:
            self.print(message_part + '\n')  # Print the initial message
        # The rest (after '$') starts the numeric data, which may be partial
        data_part_accum = received_message[msg_end_index + 1:]

        # Next, read numeric data in batches until the end-of-data indicator '#' is received
        timeout_cnt = 0
        while '#' not in data_part_accum:
            try:
                if self.serial_obj.inWaiting() > 0:
                    rdata = self.serial_obj.read(self.serial_obj.inWaiting())
                    rdata = rdata.decode('utf-8')
                    data_part_accum += rdata
                else:
                    time.sleep(0.001)
                    timeout_cnt += 1
                    if timeout_cnt >= self.serial_timeout:
                        self.error('Serial: Timeout waiting for data part')
                        self.status(False)
                        self.serial_ready = True
                        raise TimeoutError("Timeout waiting for data part")
            except Exception as e:
                print(str(e))
                self.error("Serial: Read Data Failed")
                self.status(False)
                self.serial_ready = True
                raise RuntimeError("Error reading data part") from e

        # Extract the numeric data string up to the '#' marker
        data_end_index = data_part_accum.find('#')
        numeric_data_str = data_part_accum[:data_end_index]
        print(f"Numeric data received: {numeric_data_str}")

        # ------------------------------
        # Decode the Numeric Data
        # It may be a single integer (e.g., "666") or multiple values (e.g., "1,2,3" or "[1,2,3]")
        # ------------------------------
        try:
            cleaned = numeric_data_str.strip()
            if not cleaned:
                data = None
            elif cleaned.startswith('['):
                # If it's bracketed, safely evaluate the list.
                data = ast.literal_eval(cleaned)
            elif ',' in cleaned:
                # If there are commas, split and convert to integers.
                data = [int(x) for x in cleaned.split(',') if x.strip() != '']
            else:
                # Otherwise, treat it as a single integer.
                data = int(cleaned)
            print(f"Decoded data: {data}")
        except Exception as e:
            self.error('Serial: Decoding Failed')
            print(f"Numeric data string: {numeric_data_str}")
            raise SystemError("Decoding failed") from e

        # ------------------------------
        # Leave the Critical Region and Return the Parsed Data
        # ------------------------------
        self.status(False)
        self.serial_ready = True
        return data



if __name__ == "__main__":
    mcu_debug = DebugCommand()
    mcu_debug.print("\n")
    mcu_debug.print("---- Command Line Interface for Instrument ----\n")
    mcu_debug.print("---- S: Message from system\n")
    mcu_debug.print("---- M: Message from MCU\n")
    mcu_debug.print("S: type your command, e.g., 'list_serial'\n\n")
    mcu_debug.print("S: scanning the serial port... wait !\n")
    mcu_debug.execute_cmd('list_serial')

    while True:
        time.sleep(0.01)
        input_cmd = input(">> ")
        # noinspection PyBroadException
        try:
            res = mcu_debug.execute_cmd(input_cmd.lower())
        except TimeoutError:
            mcu_debug.print("S: Timeout.\n")
        except:
            pass
