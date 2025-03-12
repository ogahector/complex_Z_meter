# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from PyQt6.QtCore import pyqtSignal, QThread

# --------------------------------------------------------------
# User Import
# --------------------------------------------------------------
from math import *
from Instrument_Func import *

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
            status_pos12 = self.serial_obj.execute_cmd('check_status_pos12')
            status_neg12 = self.serial_obj.execute_cmd('check_status_neg12')
            status_3v3 = self.serial_obj.execute_cmd('check_status_3v3')
        except Exception as exc:
            print("Init: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, status_pos12, status_neg12, status_3v3])
        # ----------------------------------------------------------------------------------------------------------------------
        self.serial_obj.serial_timeout = 100000000
        # Reset
        self.next_step = False

    def config(self, file_path, next_step):
        self.file_path = file_path
        self.next_step = next_step


# ==============================================================================================================================
# Open Calibration
# ==============================================================================================================================
class ShortCalibration(QThread):
    done_s = pyqtSignal(list)

    file_path = None
    next_step = False

    def __init__(self, serial, plot, parent=None):
        super(ShortCalibration, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot
        self.received_phasors = []  # Store received phasors

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # Start the calibration loop
            while True:
                sc_phasors = self.serial_obj.execute_cmd("start_sc_calib")
                if sc_phasors is None:
                    print("sc_phasors is None")
                    break  # End loop if no more phasors

                # Add received phasor to the list
                self.received_phasors.append(sc_phasors[0])

                # Console output and save to binary file
                self.serial_obj.print(f'SC Calibration Phasors {sc_phasors}')
                binary_file_write_phasors(self.file_path + '\\%s_short_calibration.bin' % get_date_time(2), self.received_phasors)

                # Update the plot with the new phasor
                freq, mag, phase = sc_phasors[0]  # Unpack the phasor
                self.plot_obj.plot_bode([f for f, _, _ in self.received_phasors], [m for _, m, _ in self.received_phasors], [p for _, _, p in self.received_phasors], option='sc_calib')

                # Check for stop signal (e.g., stop button pressed)
                if self.next_step:  # Check for stop condition
                    self.serial_obj.execute_cmd("stop_sc_calib")
                    break

                time.sleep(1)

        except Exception as exc:
            print(f"ShortCalibration Error: {exc}")
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, self.received_phasors])

        self.serial_obj.serial_timeout = 1000000
        self.next_step = False

    def config(self, file_path, next_step=False):
        self.file_path = file_path
        self.next_step = next_step

# ==============================================================================================================================
# Open Calibration
# ==============================================================================================================================
class OpenCalibration(QThread):
    done_s = pyqtSignal(list)

    file_path = None
    next_step = False

    def __init__(self, serial, plot, parent=None):
        super(OpenCalibration, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        # ----------------------------------------------------------------------------------------------------------------------
        try:
            # Cmd
            oc_phasors = self.serial_obj.execute_cmd("start_oc_calib")
            if oc_phasors is None:
                print("oc_phasors is None")

            # Console
            self.serial_obj.print('\nS: OC Calib Phasors %s, ' % oc_phasors)

            # Save data to a binary file
            binary_file_write_phasors(self.file_path + '\\%s_open_calibration.bin' % get_date_time(2), oc_phasors)

            # Extract magnitude and phase from sc_phasors
            # Assuming sc_phasors is a list of tuples [(magnitude1, phase1), (magnitude2, phase2), ...]
            magnitudes = [phasor[0] for phasor in oc_phasors]
            phases = [phasor[1] for phasor in oc_phasors]

            # Update the Bode plot
            # We use a dashed line style for each phasor's plot
            self.plot_obj.plot_bode([10, 100, 1000, 10000], magnitudes,
                                    phases, option='oc_calib')  # Use an empty frequency array if frequencies are not available

        except Exception as exc:
            print("OpenCalibration: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, oc_phasors])
        # ----------------------------------------------------------------------------------------------------------------------
        self.serial_obj.serial_timeout = 1000000
        # Reset
        self.next_step = False

    def config(self, file_path, next_step=False):
        self.file_path = file_path
        self.next_step = next_step


# ==============================================================================================================================
# Real-time Measurement Readout
# ==============================================================================================================================
class ReadoutMeasurement(QThread):
    update_s = pyqtSignal(list)
    done_s = pyqtSignal(list)

    file_path = None
    next_step = False

    def __init__(self, serial, plot, parent=None):
        super(ReadoutMeasurement, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot

    def run(self):
        try:
            # Step 1: Get the phasors (magnitude, phase) for the measurement
            dut_phasors = self.serial_obj.execute_cmd("readout_meas")

            if dut_phasors is None:
                print("dut_phasors is None")

            # Step 2: Console print the received phasors (you can modify this if you need more details)
            self.serial_obj.print('\nS: Readout Measurement Phasors %s, ' % dut_phasors)

            # Step 3: Save the data to a binary file
            binary_file_write_phasors(self.file_path + '\\%s_readout_measurement.bin' % get_date_time(2),
                                      dut_phasors)

            # Step 4: Extract magnitude and phase from the received phasors
            magnitudes = [phasor[0] for phasor in dut_phasors]
            phases = [phasor[1] for phasor in dut_phasors]

            # Step 5: Update the Bode plot with the new data
            # We use a frequency array and update the plot with dashed lines for measurement data
            self.plot_obj.plot_bode([10, 100, 1000, 10000], magnitudes, phases, option='meas')

            # Step 6: Emit the updated data to the UI (if needed)
            self.update_s.emit(dut_phasors)

            time.sleep(1)  # Delay between readouts

        except Exception as exc:
            print(f"ReadoutMeasurement Error: {exc}")
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, dut_phasors])
        # ----------------------------------------------------------------------------------------------------------------------
        self.serial_obj.serial_timeout = 1000000
        # Reset
        self.next_step = False

    def config(self, file_path, next_step=False):
        self.file_path = file_path
        self.next_step = next_step


# ==============================================================================================================================
# RLC Fitting
# ==============================================================================================================================
class RLCFitting(QThread):
    done_s = pyqtSignal(list)

    file_path = None
    next_step = False

    def __init__(self, serial, plot, parent=None):
        super(RLCFitting, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # Cmd
            rlc_data = self.serial_obj.execute_cmd("start_rlc_fit")
            if rlc_data is None:
                print("rlc_data is None")
            # Console
            self.serial_obj.print('\nS: RLC Fit Data %s, ' % rlc_data)
            # Save
            binary_file_write(self.file_path + '\\%s_rlc_fitting.bin' % get_date_time(2), rlc_data)
        except Exception as exc:
            print("RLCFitting: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, rlc_data[-1]])
        self.serial_obj.serial_timeout = 1000000
        self.next_step = False

    def config(self, file_path, next_step=False):
        self.file_path = file_path
        self.next_step = next_step


# ==============================================================================================================================
# RLC Fitting
# ==============================================================================================================================
class QFactor(QThread):
    done_s = pyqtSignal(list)

    file_path = None
    next_step = False

    def __init__(self, serial, plot, parent=None):
        super(QFactor, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # Cmd
            qf_data = self.serial_obj.execute_cmd("start_q_factor")
            if qf_data is None:
                print("qf_data is None")
            # Console
            self.serial_obj.print('\nS: Q Factor Data %s, ' % qf_data)
            # Save
            binary_file_write(self.file_path + '\\%s_q_factor.bin' % get_date_time(2), qf_data)
        except Exception as exc:
            print("QFactor: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, qf_data[-1]])
        self.serial_obj.serial_timeout = 1000000
        self.next_step = False

    def config(self, file_path, next_step=False):
        self.file_path = file_path
        self.next_step = next_step


# ==============================================================================================================================
# Power Status Check
# - Queries the status of +12V, -12V, and 3.3V power rails
# ==============================================================================================================================
class CheckPowerStatus(QThread):
    done_s = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(CheckPowerStatus, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            pos12 = self.serial_obj.execute_cmd("check_status_pos12")
            neg12 = self.serial_obj.execute_cmd("check_status_neg12")
            v3_3 = self.serial_obj.execute_cmd("check_status_3v3")
        except Exception as exc:
            print(f"CheckPowerStatus Error:" + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([pos12, neg12, v3_3])
        self.serial_obj.serial_timeout = 1000000


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
        except Exception as exc:
            print(f"DACControl Error: {exc}")
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
        except Exception as exc:
            print(f"ADCRead Error: {exc}")
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
        except Exception as exc:
            print(f"CurrentRead Error: {exc}")
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
        except Exception as exc:
            print(f"ReferenceResistor Error: {exc}")
            self.done_s.emit([None])


# ==============================================================================================================================
# Serial
# ==============================================================================================================================
# Serial general command -------------------------------------------------------------------------------------------------------
class SerialGeneralCmd(QThread):
    done_s = None

    command = None
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
        except Exception as exc:
            print("SerialGeneralCmd: " + str(exc))

        self.serial_obj.serial_timeout = 1000000


# Open a serial port -----------------------------------------------------------------------------------------------------------
class SerialOpenPort(QThread):
    done_s = pyqtSignal(list)
    serial_port = None  # MIGHT NEED TO REMOVE THIS

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
            device_id = self.serial_obj.execute_cmd("get_id")
            # Clock frequency
            div = self.serial_obj.execute_cmd("get_clk_divpw")
        except Exception as exc:
            print("SerialOpenPort: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([ver, device_id, div])

    def config(self, serial_port):
        self.serial_port = serial_port


# List available serial ports --------------------------------------------------------------------------------------------------
class SerialListPort(QThread):
    done_s = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(SerialListPort, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        (port_name, port_list) = self.serial_obj.list_serial()
        self.done_s.emit([port_name, port_list])
