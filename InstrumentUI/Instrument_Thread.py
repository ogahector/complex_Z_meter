# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from PyQt6.QtCore import pyqtSignal, QThread
import pydevd

# --------------------------------------------------------------
# User Import
# --------------------------------------------------------------
from math import *
from Instrument_Func import *
import rlc_fitting_re_im as rlc

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
# ==============================================================================================================================
class Init(QThread):
    done_s = pyqtSignal(list)

    file_path = None

    def __init__(self, serial, parent=None):
        super(Init, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        self.serial_obj.serial_timeout = 100000000
        # ----------------------------------------------------------------------------------------------------------------------
        try:
            # This cmd includes reset & init
            status_pos12 = self.serial_obj.execute_cmd('check_status_pos12')
            status_neg12 = self.serial_obj.execute_cmd('check_status_neg12')
            status_3v3 = self.serial_obj.execute_cmd('check_status_3v3')
        except Exception as exc:
            print("Init: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, status_pos12, status_neg12, status_3v3])
        # ----------------------------------------------------------------------------------------------------------------------
        self.serial_obj.serial_timeout = 100000000

    def config(self, file_path):
        self.file_path = file_path

# ==============================================================================================================================
# Short Calibration
# ==============================================================================================================================
class ShortCalibration(QThread):
    done_s = pyqtSignal(list)

    file_path = None

    def __init__(self, serial, plot, parent=None):
        super(ShortCalibration, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot
        self.received_phasors = []  # Store received phasors
        self.run = True

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # Start the calibration loop
            sc_phasors = self.serial_obj.execute_cmd("start_sc_calib")

            # Check if no phasor is returned
            if sc_phasors[0] is None:
                print("No more phasors received. Saving data to file. Ending short circuit calibration.")
                # Save to binary file
                write_phasors_to_text_file(self.file_path + '\\%s_short_calibration.txt' % get_date_time(2),
                                            self.received_phasors)
                self.stop()

            frequencies = []
            magnitudes = []
            phases = [] # convert to degs
            for i in range(len(sc_phasors)):
                if i % 3 == 0:
                    frequencies.append(sc_phasors[i])
                if i % 3 == 1:
                    magnitudes.append(sc_phasors[i])
                if i % 3 == 2:
                    phases.append(sc_phasors[i])

            # Add the received phasor to the list
            self.received_phasors = [ [frequencies[i], 20*np.log10(magnitudes[i]), (180/np.pi) * phases[i]] for i in range(len(phases)) ]

            # v_phasors = np.array(magnitudes) * np.exp(1j * phases)
            # z_phasors = 100 * v_phasors / (1 - v_phasors)

            # Update the plot with the new phasor
            self.plot_obj.plot_bode([f for f, _, _ in self.received_phasors],
                                        [m for _, m, _ in self.received_phasors],
                                        [p for _, _, p in self.received_phasors], option='sc_calib')
            time.sleep(1)

        except Exception as exc:
            print(f"ShortCalibration Error: {exc}")
            self.done_s.emit([])  # Emit empty result on error
        else:
            self.done_s.emit([self.file_path, self.received_phasors])

        # Reset after process ends
        self.serial_obj.serial_timeout = 1000000

    def config(self, file_path):
        self.file_path = file_path
        self.received_phasors = []  # Store received phasors
        self.run = True

    def stop(self):
        self.serial_obj.execute_cmd("stop_sc_calib")
        self.run = False  # Set run False to exit the loop

# ==============================================================================================================================
# Open Calibration
# ==============================================================================================================================
class OpenCalibration(QThread):
    done_s = pyqtSignal(list)

    file_path = None

    def __init__(self, serial, plot, parent=None):
        super(OpenCalibration, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot
        self.received_phasors = []  # Store received phasors
        self.run = True

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # Start the calibration loop
            # Get the phasors (frequency, magnitude, phase)
            oc_phasors = self.serial_obj.execute_cmd("start_oc_calib")

            # Check if no phasor is returned
            if oc_phasors[0] is None:
                print("No more phasors received. Saving data to file. Ending open circuit calibration.")
                # Save to binary file
                write_phasors_to_text_file(self.file_path + '\\%s_open_calibration.txt' % get_date_time(2), self.received_phasors)

                self.stop()

            frequencies = []
            magnitudes = []
            phases = [] # convert to degs
            for i in range(len(oc_phasors)):
                if i % 3 == 0:
                    frequencies.append(oc_phasors[i])
                if i % 3 == 1:
                    magnitudes.append(oc_phasors[i])
                if i % 3 == 2:
                    phases.append(oc_phasors[i])

            # Add the received phasor to the list
            self.received_phasors = [ [frequencies[i], 20*np.log10(magnitudes[i]), (180/np.pi) * phases[i]] for i in range(len(phases)) ]

            # v_phasors = np.array(magnitudes) * np.exp(1j * phases)
            # z_phasors = 100 * v_phasors / (1 - v_phasors)

            # Update the plot with the new phasor
            self.plot_obj.plot_bode([f for f, _, _ in self.received_phasors],
                                        [m for _, m, _ in self.received_phasors],
                                        [p for _, _, p in self.received_phasors], option='oc_calib')

        except Exception as exc:
            print(f"OpenCalibration Error: {exc}")
            self.done_s.emit([])  # Emit empty result on error
        else:
            self.done_s.emit([self.file_path, self.received_phasors])

        # Reset after process ends
        self.serial_obj.serial_timeout = 1000000

    def config(self, file_path):
        self.file_path = file_path
        self.received_phasors = []  # Store received phasors
        self.run = True

    def stop(self):
        self.serial_obj.execute_cmd("stop_oc_calib")
        self.run = False  # Set run False to exit the loop

# ==============================================================================================================================
# Load Calibration
# ==============================================================================================================================
class LoadCalibration(QThread):
    done_s = pyqtSignal(list)

    file_path = None

    def __init__(self, serial, plot, parent=None):
        super(LoadCalibration, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot
        self.received_phasors = []  # Store received phasors
        self.run = True

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # Start the calibration loop
            # Get the phasors (frequency, magnitude, phase)
            oc_phasors = self.serial_obj.execute_cmd("start_ld_calib")

            # Check if no phasor is returned
            if oc_phasors[0] is None:
                print("No more phasors received. Saving data to file. Ending load calibration.")
                # Save to binary file
                write_phasors_to_text_file(self.file_path + '\\%s_load_calibration.txt' % get_date_time(2), self.received_phasors)

                self.stop()

            frequencies = []
            magnitudes = []
            phases = [] # convert to degs
            for i in range(len(oc_phasors)):
                if i % 3 == 0:
                    frequencies.append(oc_phasors[i])
                if i % 3 == 1:
                    magnitudes.append(oc_phasors[i])
                if i % 3 == 2:
                    phases.append(oc_phasors[i])

            # Add the received phasor to the list
            self.received_phasors = [ [frequencies[i], 20*np.log10(magnitudes[i]), (180/np.pi) * phases[i]] for i in range(len(phases)) ]

            # v_phasors = np.array(magnitudes) * np.exp(1j * phases)
            # z_phasors = 100 * v_phasors / (1 - v_phasors)

            # Update the plot with the new phasor
            self.plot_obj.plot_bode([f for f, _, _ in self.received_phasors],
                                        [m for _, m, _ in self.received_phasors],
                                        [p for _, _, p in self.received_phasors], option='load_calib')

        except Exception as exc:
            print(f"LoadCalibration Error: {exc}")
            self.done_s.emit([])  # Emit empty result on error
        else:
            self.done_s.emit([self.file_path, self.received_phasors])

        # Reset after process ends
        self.serial_obj.serial_timeout = 1000000

    def config(self, file_path):
        self.file_path = file_path
        self.received_phasors = []  # Store received phasors
        self.run = True

    def stop(self):
        self.serial_obj.execute_cmd("stop_ld_calib")
        self.run = False  # Set run False to exit the loop

# ==============================================================================================================================
# Real-time Measurement Readout
# ==============================================================================================================================
class ReadoutMeasurement(QThread):
    update_s = pyqtSignal(list)
    done_s = pyqtSignal(list)

    file_path = None

    def __init__(self, serial, plot, parent=None):
        super(ReadoutMeasurement, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot
        self.received_phasors = []  # Store received phasors
        self.run = True

    def run(self):
        try:
            # Start the measurement loop
            # Get the phasors (frequency, magnitude, phase) for the measurement
            dut_phasors = self.serial_obj.execute_cmd("readout_meas")
            # print(np.size(dut_phasors))

            # Check if no phasor is returned
            if dut_phasors[0] is None: # idk what the top condition would be?
                print("No more phasors received. Saving data to file. Ending readout measurement.")
                # Save to binary file
                write_phasors_to_text_file(self.file_path + '\\%s_readout_measurement.txt' % get_date_time(2),
                                            self.received_phasors)

                self.done_s.emit([self.file_path, self.received_phasors])
                self.stop()

            frequencies = []
            magnitudes = []
            phases = [] # convert to degs
            for i in range(len(dut_phasors)):
                if i % 3 == 0:
                    frequencies.append(dut_phasors[i])
                if i % 3 == 1:
                    magnitudes.append(dut_phasors[i])
                if i % 3 == 2:
                    phases.append(dut_phasors[i])

            # Add the received phasor to the list
            self.received_phasors = [ [frequencies[i], 20*np.log10(magnitudes[i]), (180/np.pi) * phases[i]] for i in range(len(phases)) ]

            # v_phasors = np.array(magnitudes) * np.exp(1j * phases)
            # z_phasors = 100 * v_phasors / (1 - v_phasors)

            # Update the plot with the new phasor
            self.plot_obj.plot_bode([f for f, _, _ in self.received_phasors],
                                        [m for _, m, _ in self.received_phasors],
                                        [p for _, _, p in self.received_phasors], option='meas')


            # Emit the updated data to the UI (if needed)
            self.update_s.emit(dut_phasors)

            time.sleep(1)  # Delay between readouts

        except Exception as exc:
            print(f"ReadoutMeasurement Error: {exc}")
            self.done_s.emit([])  # Emit empty result on error
        else:
            self.done_s.emit([self.file_path, self.received_phasors])

        # Reset after process ends
        self.serial_obj.serial_timeout = 1000000

    def config(self, file_path):
        self.file_path = file_path
        self.received_phasors = []  # Store received phasors
        self.run = True

    def stop(self):
        self.serial_obj.execute_cmd("stop_readout_meas")
        self.run = False  # Set run False to exit the loop

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
            # MCU will just resend what it has stored - just makes it easier
            rlc_data = self.serial_obj.execute_cmd("start_rlc_fit")
            if rlc_data is None:
                print("rlc_data is None")
            # Note: The file saving routine is commented out since it's not working
            # binary_file_write(self.file_path + '\\%s_rlc_fitting.txt' % get_date_time(2), rlc_data)

            # Extract frequencies from rlc_data (assuming triplets: [freq, mag, phase, ...])
            freqs = np.array([rlc_data[i] for i in range(len(rlc_data)) if i % 3 == 0])

            # Perform the RLC fit; rlc.rlc_fit_re_im returns a name and a best_fit dict
            name, best_fit = rlc.rlc_fit_re_im(rlc_data, Rtol=0.1, Ltol=1e-9, Ctol=1e-12)

            try:
                fitfunc = best_fit["model"]
                RLCparams = best_fit["popt"]
            except KeyError as e:
                print("Key error? What did you send through??")
                # Optionally, you may want to exit the thread here
                self.done_s.emit([])
                return

            # Compute theoretical impedance using the fitted model
            Z_theoretical = fitfunc(freqs, *RLCparams)

            # --- Percentage Error Calculation ---
            # Extract measured impedance from rlc_data.
            # We assume that the data are arranged in triplets: [frequency, mag, phase, ...]
            Z_measured = np.array(
                [rlc_data[i+1] * np.exp(1j * rlc_data[i+2])
                 for i in range(0, len(rlc_data), 3)]
            )
            # Calculate percentage error in logarithmic scale
            mag_percentage_error = np.abs(
                (np.log10(np.abs(Z_measured)) - np.log10(np.abs(Z_theoretical))) /
                np.log10(np.abs(Z_theoretical))
            ) * 100

            phase_percentage_error = np.abs(
                (np.angle(Z_theoretical) - np.angle(Z_measured)) /
                np.angle(Z_theoretical)
            ) * 100

            # Plot the error using the provided function.
            # In this context, 'time' is taken as the frequency axis.
            self.plot_obj.plot_rlc_fit(freqs, mag_percentage_error, phase_percentage_error)

            # Plot the theoretical bode plot
            self.plot_obj.plot_bode(
                freqs,
                20 * np.log10(np.abs(Z_theoretical)),
                180 / np.pi * np.unwrap(np.angle(Z_theoretical)),
                option='model_fit'
            )

            # Print device and parameter information
            self.serial_obj.print(
                f'RLC Device: {name}, with R: {RLCparams[0]:.3e} L: {RLCparams[1]:.3e} C: {RLCparams[2]:.3e}'
            )

        except Exception as exc:
            print("RLCFitting: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, rlc_data[-1]])
        self.serial_obj.serial_timeout = 1000000
        self.next_step = True

    def config(self, file_path, next_step=False):
        self.file_path = file_path
        self.next_step = next_step

# ==============================================================================================================================
# Q Factor
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
            binary_file_write(self.file_path + '\\%s_q_factor.txt' % get_date_time(2), qf_data)
        except Exception as exc:
            print("QFactor: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step, qf_data[-1]])
        self.serial_obj.serial_timeout = 1000000
        self.next_step = False

    def config(self, file_path, next_step=False):
        self.file_path = file_path
        # self.next_step = next_step
        self.next_step = True


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
                print(self.value)
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
            clk = self.serial_obj.execute_cmd("get_clk_divpw")
        except Exception as exc:
            print("SerialOpenPort: " + str(exc))
            self.done_s.emit([])
        else:
            self.done_s.emit([ver, device_id, clk])

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
