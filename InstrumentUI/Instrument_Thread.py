# --------------------------------------------------------------
# UI Import
# --------------------------------------------------------------
from PyQt6.QtCore import pyqtSignal, QThread

# --------------------------------------------------------------
# User Import
# --------------------------------------------------------------
from math import *

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
#                                   Check Gain
# ==============================================================================================================================

class Check_Gain(QThread):
    done_s      = pyqtSignal(list)

    file_path     = None
    next_step     = False

    def __init__(self, serial, plot, parent=None):
        super(Check_Gain, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # --------------------------------
            # Init
            out_well = []
            time_start      = time.time()
            time_stamp      = []
            file_name    = '\%s_gain.bin' % get_date_time(2)

            for frame in range(0, 8):
                # Readout, plot and save
                try:
                    read_from_mcu = self.serial_obj.execute_cmd('check_gain 1')
                except Exception as e:
                    print("Check_Gain IMAGE: " + str(e))
                    time.sleep(1)
                    continue

                if(len(read_from_mcu)<P):
                    time.sleep(1)
                    print("len<59160")
                    continue
                    
                out_time_1D = read_from_mcu[0:P]
                extra_bytes = read_from_mcu[P:]
                Vref = extra_bytes[0]
                    
                try:
                    # Plot image
                    self.plot_obj.plot_image(out_time_1D)
                    # Timestamp
                    time_stamp.append(time.time() - time_start)
                    # Plot all well information
                    n_wells = int(len(extra_bytes[0:])/2)-1
                    # Well outputs
                    out_well.append(extra_bytes[1:n_wells+2][1:])
                    # Index of active wells
                    self.plot_obj.plot_cur(out_well, time_stamp)
                    # Draw
                    self.plot_obj.drawnow()
                    
                    if (frame==3):
                        print(extra_bytes[n_wells+1:][1:])
                except Exception as e:
                    print("Check_Gain DATA: " + str(e))
                    continue
                    
                # Append data
                for i in range(0, n_wells):
                    out_time_1D.append(out_well[frame][i])     # Timestamp, /10 = ms
                # Save into a single binary file
                binary_file_append(self.file_path + file_name, out_time_1D)
            # --------------------------------
        except Exception as e:
            print("Check_Gain: " + str(e))
            self.done_s.emit([])
        else:
            self.done_s.emit([self.file_path, self.next_step])

        self.serial_obj.serial_timeout = 1000000
        
        # Reset
        self.next_step = False
        
    def config(self, file_path, next_step=False):
        self.file_path = file_path
        self.next_step = next_step

# ================================================================================================================================
# Readout Time - Legacy Heat
# ================================================================================================================================
class Readout_Time(QThread):
    update_s    = pyqtSignal(list)
    done_s      = pyqtSignal(list)

    file_name     = None
    file_path     = None
    next_step     = False

    def __init__(self, serial, plot, parent=None):
        super(Readout_Time, self).__init__(parent)
        self.serial_obj = serial
        self.plot_obj = plot

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        try:
            # --------------------------------
            # Init
            self.run        = True
            cur_data    = []
            out_well = []
            tem_data    = []
            tem_data_lin    = []
            irf_data    = []
            time_start      = time.time()
            time_stamp      = []
            file_name    = '\%s_readout_time.bin' % get_date_time(2)
            # Extra bytes per frame
            #E = 10 # First is average temp output, second is averaged linearised output

            while(self.run):
                # Readout, plot and save
                try:
                    #pwm_temp = self.param # To delete if no parameter needed
                    heat = self.param
                    print('Heat value is')
                    print(heat)
                    read_from_mcu = self.serial_obj.execute_cmd('readout_time 1')
                except Exception as e:
                    print("Readout_Time IMAGE: " + str(e))
                    time.sleep(1)
                    continue
                
                if(len(read_from_mcu)<P):
                    time.sleep(1)
                    print("len<59160")
                    continue
                
                out_time_1D = read_from_mcu[0:P]
                extra_bytes = read_from_mcu[P:]
                vref = extra_bytes[0]
                
                try:
                    # Plot image
                    self.plot_obj.plot_image(out_time_1D)
                    # Timestamp
                    time_stamp.append(time.time() - time_start)
                    cur_data.append(round(extra_bytes[1]/10))
                    
                    # Plot temperature vs
                    tem = round(extra_bytes[2]/10)
                    tem_lin = round(extra_bytes[3])-100 # Gain of 10
                    # Import parameters of temp regulation
                    reg_err = round(extra_bytes[4]/10)
                    reg_P = round(extra_bytes[5]/10)
                    reg_I = round(extra_bytes[6]/10)
                    reg_D = round(extra_bytes[7]/10)
                    reg_ref = round(extra_bytes[8]/10)
                    frame_temp_ok = extra_bytes[9]
                    frame_temp_st = extra_bytes[10]
                    time_temp_ok = time_stamp[extra_bytes[9]]
                    time_temp_st = time_stamp[extra_bytes[10]]
                    time_curr = time_stamp[-1]
                    # Plot all well information
                    n_wells = int(len(extra_bytes[11:])/2)-1
                    # Well outputs
                    out_well.append(extra_bytes[11:11+n_wells+1][1:])
                    # Number of active pixels in well
                    well_pixel_on = extra_bytes[11+n_wells+1:11+2*(n_wells+1)]
                    # Index of active wells
                    out = self.serial_obj.execute_cmd("get_active_wells")
                    self.plot_obj.plot_cur(out_well, time_stamp)
                    # Plot temperature pixel
                    tem_data.append(tem)
                    tem_data_lin.append(tem_lin) # To avoid negative
                    self.plot_obj.plot_tem(tem_data_lin, time_stamp)
                    # Draw
                    self.plot_obj.drawnow()
                    # Record data
                    adc = self.serial_obj.execute_cmd('adc_get_val')
                    pwm = self.serial_obj.execute_cmd('tim2_get_pulse')
                    tam = self.serial_obj.execute_cmd('get_ambient_temp')
                    tch = None
                    soc = None
                    cur = None
                    vol = None
                except Exception as e:
                    print("Readout_Time DATA: " + str(e))
                    continue

                self.update_s.emit([None] + [tem, tem_lin, reg_err, reg_P, reg_I, reg_D, reg_ref, frame_temp_ok, frame_temp_st, time_temp_ok,time_temp_st,time_curr, well_pixel_on, vref,  adc, pwm, tam, tch] + [soc, cur, vol])

                # Append data
                out_time_1D.append(9)     # Number of parameters
                out_time_1D.append(int(time_stamp[-1]*10) & 0xFFFF)     # Timestamp, /10 = ms
                out_time_1D.append(vref)                         # Vref
                out_time_1D.append(cur_data[-1])                    # Average chem readout
                out_time_1D.append(tem_data[-1])                    # Average temp readout
                out_time_1D.append(tem_lin+100)                         # Average linearised temp readout # To avoid negative
                out_time_1D.append(adc if adc else 0xFFFF)              # ADC
                out_time_1D.append(tam if tam else 0xFFFF)              # Ambient temperature
                out_time_1D.append(tch if tch else 0xFFFF)              # Chamber temperature

                # Save into a single binary file
                binary_file_append(self.file_path + file_name, out_time_1D)
            # --------------------------------
        except Exception as e:
            print("Readout_Time: " + str(e))
            self.done_s.emit([self.file_path, None])
        else:
            self.done_s.emit([self.file_path])

        self.serial_obj.serial_timeout = 1000000
        
    def stop(self):
        print("Stop readout")
        print("Turn off heater")
        self.serial_obj.execute_cmd('tim2_set_pulse 0')
        self.run = False
        
    def config(self, file_path, next_step=False,  param=0):
        self.file_path = file_path
        self.next_step = next_step
        self.param = param

# ==============================================================================================================================
# Real-time Current Sensing for Reference Electrode
# ==============================================================================================================================
class _Iref_Monitor(QThread):
    update_s = pyqtSignal()
    
    run = False 

    def __init__(self, parent=None):
        super(_Iref_Monitor, self).__init__(parent)

    def run(self):
        self.run = True 
        while(self.run):
            self.sleep(1)
            self.update_s.emit()

    def stop(self):
        self.run = False

# ==============================================================================================================================
# Serial
# ==============================================================================================================================
# Serial general command -------------------------------------------------------------------------------------------------------
class Serial_General_Cmd(QThread):
    done_s = None

    command  = None
    response = True
    
    def __init__(self, serial, parent=None):
        super(Serial_General_Cmd, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        self.serial_obj.serial_timeout = 10000000
        
        try:
            r = self.serial_obj.execute_cmd(self.command)
            if(self.response):
                self.done_s.emit("R: %s\n" % (r if type(r) is not list else str(r)))
        except Exception as e:
            print("Serial_General_Cmd: " + str(e))
            
        self.serial_obj.serial_timeout = 1000000

# Open a serial port -----------------------------------------------------------------------------------------------------------
class Serial_Open_Port(QThread):
    done_s   = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(Serial_Open_Port, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        try:
            # Open serial
            self.serial_obj.open_serial(self.serial_port)
            self.serial_obj.clear()
            # Firmware version
            ver = self.serial_obj.execute_cmd("get_version")
            id  = self.serial_obj.execute_cmd("get_id")
            # Clock frequency
            div = self.serial_obj.execute_cmd("_get_clk_divpw")
            dac = self.serial_obj.execute_cmd("dac_get_val")
        except Exception as e:
            print("Serial_Open_Port: " + str(e))
            self.done_s.emit([])
        else:
            self.done_s.emit([ver, id, div, dac])

    def config(self, serial_port):
        self.serial_port = serial_port
    
# List available serial ports --------------------------------------------------------------------------------------------------
class Serial_List_Port(QThread):
    done_s   = pyqtSignal(list)

    def __init__(self, serial, parent=None):
        super(Serial_List_Port, self).__init__(parent)
        self.serial_obj = serial

    def run(self):
        (port_name, port_list) = self.serial_obj.list_serial()
        self.done_s.emit([port_name, port_list])
