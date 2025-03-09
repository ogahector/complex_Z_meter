# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from PyQt6.QtCore import pyqtSignal, QThread

# --------------------------------------------------------------
# User Import
# --------------------------------------------------------------
from math               import *
from Instrument_Func    import *

# --------------------------------------------------------------
# System Import
# --------------------------------------------------------------
import struct
import numpy as np
import time
import os
import cv2
import matplotlib.pyplot as plt

# ==============================================================================================================================
# Main Process
# ==============================================================================================================================
# ==============================================================================================================================
# Initialisation
class Init(QThread):
    done_s = pyqtSignal(list)
    
    file_path = None
    next_step = False
    
    def __init__(self, serial, parent=None):
        super(Init, self).__init__(parent)
        self.serial_obj = serial
        
    def run(self):
        self.serial_obj.serial_timeout = 100000000
        # ----------------------------------------------------------------------------------------------------------------------
        try:
            # This cmd includes resent & init
            status = self.serial_obj.execute_cmd('check_status_elec')
        except Exception as e:
            print("Init: " + str(e))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, status])
        # ----------------------------------------------------------------------------------------------------------------------
        self.serial_obj.serial_timeout = 100000000
        # Reset
        self.next_step = False
        
    def config(self, file_path, next_step):
        self.file_path = file_path
        self.next_step = next_step


# ==============================================================================================================================
# Power Status Check
# - Queries the status of +12V, -12V, and 3.3V power rails
# ==============================================================================================================================
class CheckPowerStatus(QThread):
    done_s = pyqtSignal(dict)

    def __init__(self, serial, parent=None):
        super(CheckPowerStatus, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        try:
            pos12 = self.serial_obj.execute_cmd("check_status_pos12")
            neg12 = self.serial_obj.execute_cmd("check_status_neg12")
            v3_3 = self.serial_obj.execute_cmd("check_status_3v3")
            self.done_s.emit({"+12V": pos12, "-12V": neg12, "3.3V": v3_3})
        except Exception as e:
            print(f"CheckPowerStatus Error: {e}")
            self.done_s.emit({"+12V": None, "-12V": None, "3.3V": None})


# ==============================================================================================================================
# Real-time Measurement Readout
# ==============================================================================================================================
class ReadoutMeasurement(QThread):
    update_s = pyqtSignal(list)
    done_s = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(ReadoutMeasurement, self).__init__(parent)
        self.serial_obj = serial
        self.run_readout = False

    def run(self):
        self.run_readout = True
        while self.run_readout:
            try:
                data = self.serial_obj.execute_cmd("readout_time")
                self.update_s.emit(data)
                time.sleep(1)  # Delay between readouts
            except Exception as e:
                print(f"ReadoutMeasurement Error: {e}")
                self.done_s.emit([])
                break

    def stop(self):
        self.run_readout = False


# ==============================================================================================================================
# DAC and ADC Control
# ==============================================================================================================================
class DACControl(QThread):
    done_s = pyqtSignal(list)

    def __init__(self, serial, value=None, parent=None):
        super(DACControl, self).__init__(parent)
        self.serial_obj = serial
        self.value = value

    def run(self):
        try:
            if self.value is not None:
                self.serial_obj.execute_cmd("dac_set_val", self.value)
            dac_value = self.serial_obj.execute_cmd("dac_get_val")
            self.done_s.emit([dac_value])
        except Exception as e:
            print(f"DACControl Error: {e}")
            self.done_s.emit([None])


class ADCRead(QThread):
    done_s = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(ADCRead, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        try:
            adc_value = self.serial_obj.execute_cmd("adc_get_val")
            self.done_s.emit([adc_value])
        except Exception as e:
            print(f"ADCRead Error: {e}")
            self.done_s.emit([None])


class CurrentRead(QThread):
    done_s = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(CurrentRead, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        try:
            current_value = self.serial_obj.execute_cmd("curr_get_val")
            self.done_s.emit([current_value])
        except Exception as e:
            print(f"CurrentRead Error: {e}")
            self.done_s.emit([None])

# ==============================================================================================================================
# Reference Resistor Control
# ==============================================================================================================================
class ReferenceResistor(QThread):
    done_s = pyqtSignal(list)

    def __init__(self, serial, value=None, parent=None):
        super(ReferenceResistor, self).__init__(parent)
        self.serial_obj = serial
        self.value = value

    def run(self):
        try:
            if self.value is not None:
                self.serial_obj.execute_cmd("rref_set_val", self.value)
            rref_value = self.serial_obj.execute_cmd("rref_get_val")
            self.done_s.emit([rref_value])
        except Exception as e:
            print(f"ReferenceResistor Error: {e}")
            self.done_s.emit([None])

# ==============================================================================================================================
# Serial
# ==============================================================================================================================
# Serial general command -------------------------------------------------------------------------------------------------------
class SerialGeneralCmd(QThread):
    done_s = None

    command  = None
    response = True
    
    def __init__(self, serial, parent=None):
        super(SerialGeneralCmd, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        
        try:
            r = self.serial_obj.execute_cmd(self.command)
            if self.response:
                self.done_s.emit("R: %s\n" % (r if type(r) is not list else str(r)))
        except Exception as e:
            print("SerialGeneralCmd: " + str(e))
            
        self.serial_obj.serial_timeout = 1000000

# Open a serial port -----------------------------------------------------------------------------------------------------------
class SerialOpenPort(QThread):
    done_s   = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(SerialOpenPort, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        try:
            # Open serial
            self.serial_obj.open_serial(self.serial_port)
            self.serial_obj.clear()
            # Firmware version
            ver = self.serial_obj.execute_cmd("get_version")
            device_id  = self.serial_obj.execute_cmd("get_id")
            # Clock frequency
            div = self.serial_obj.execute_cmd("get_clk_divpw")
        except Exception as e:
            print("SerialOpenPort: " + str(e))
            self.done_s.emit([])
        else:
            self.done_s.emit([ver, device_id, div])

    def config(self, serial_port):
        self.serial_port = serial_port
    
# List available serial ports --------------------------------------------------------------------------------------------------
class SerialListPort(QThread):
    done_s   = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(SerialListPort, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        (port_name, port_list) = self.serial_obj.list_serial()
        self.done_s.emit([port_name, port_list])
