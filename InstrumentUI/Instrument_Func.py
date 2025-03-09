from datetime import datetime
import struct
import os
import zipfile
from PyQt6.QtWidgets import QMessageBox
from PyQt6.QtWidgets import QInputDialog, QLineEdit

import numpy as np

# --------------------------------------------------------------
# Date Information
# --------------------------------------------------------------
# Get current date, e.g., '20200224' -> 2020 Feb 24
def get_date():
    return datetime.now().isoformat().replace('-', '').replace(':', '')[0:8]


# Get current date and time, e.g., '20200224T141323'
def get_date_time(fraction=0):
    return datetime.now().isoformat().replace('-', '').replace(':', '')[
           0:(15 + (0 if fraction == 0 else (1 + fraction)))]


def get_std_date_time():
    return datetime.today().strftime('%Y-%m-%d %H:%M')


# --------------------------------------------------------------
# File I/O
# --------------------------------------------------------------
def binary_file_write(file, data_list):
    fw = open(file, 'wb')
    for data in data_list:
        fw.write(struct.pack('>H', data))  # 1023 -> b'\x03\xff'
    fw.close()


def binary_file_append(file, data_list):
    fw = open(file, 'ab')
    for data in data_list:
        fw.write(struct.pack('>H', data))  # 1023 -> b'\x03\xff'
    fw.close()


def binary_file_read(file):
    fw = open(file, 'rb')
    data_byte = fw.read()
    data_len = len(data_byte) >> 1
    data_list = []
    for n in range(0, data_len):
        (data,) = struct.unpack('>H', data_byte[2 * n:2 * (n + 1)])
        data_list.append(data)
    return data_list


def flip_byte(num):
    # UART DMA transmits uint16_t as little-endian '<'
    # - but i2c, spi
    num = ((num & 0xFF) << 8) | (num >> 8)


def save_to_json(folder_path, info_dict):
    fw = open(folder_path + '/info.json', 'w')
    fw.write(str(info_dict).replace("'", '"'))
    fw.close()


def zip_folder(in_path):
    # Get files
    file_list = os.listdir(in_path)
    # Out path and name
    out_path = os.path.dirname(in_path)
    out_name = os.path.basename(in_path) + '.zip'
    # Zip
    zip = zipfile.ZipFile(out_path + '/' + out_name, 'w', zipfile.ZIP_DEFLATED)
    for file in file_list:
        zip.write(in_path + '/' + file, file)
    zip.close()


def uint16_to_int16(val):
    return val if (val & 0x8000 == 0) else (val | (~0xFFFF))


# --------------------------------------------------------------
# PCB / Hardware
# --------------------------------------------------------------
# ADC to Iref
# - note that this is theoretical calculation
# - in general, abs(iref) < 150 indicates nearly zero current
def adc_val_to_iref(val):  # [nA]
    # adc
    vadc = 3.3 * val / 4095
    # current-sense amplifier
    vcs = (vadc * 1.5 - 2.488) / 90 * 1000
    # current [uA]
    iref = vcs / 10 * 10 ** 3
    return iref


# DAC to Vref
# - note that this is theoretical calculation
# - the main ui utilizes a calibrated conversion
def dac_val_to_vref(val):  # [V]
    # dac
    vdac = val * 3.3 / 4095
    # opamp
    vamp = (39 / 15 + 1) * vdac
    vref = 2 * vamp - 12
    return vref

# Read User input, accept --------------------------------------------------------------------------------------------------
# - 1) Dec: '123'
# - 2) Hex: '0xFF'
# - 3) Bin: '0b100'
def str2num(str):
    try:
        return int(str, 0)
    except:
        ui_send_error("Invalid Input: %s" % str)
        raise ValueError

def str2dec(str):
    try:
        return float(str)
    except:
        ui_send_error("Invalid Input: %f" % str)
        raise ValueError

# List files and folders under the folder path -----------------------------------------------------------------------------
def list_folder(folder_path):
    try:
        folder_list = os.listdir(folder_path)
    except:
        ui_send_error("Invalid folder path !")
        raise SystemError
    return folder_list

# Check folder existence and create ----------------------------------------------------------------------------------------
def create_folder_if_not_existed(folder_path, folder_name, ask = False):
    try:
        folder_list = list_folder(folder_path)
    except:
        if ui_send_question("Would you like to create folder %s ?" % folder_path):
            try:
                os.mkdir(folder_path)
                folder_list = list_folder(folder_path)
            except:
                ui_send_error("Invalid path for folder creation'%s'" % folder_path)
                raise IOError
        else:
            raise IOError

    if folder_name in folder_list:
        if ask:
            if not ui_send_question("Folder '%s' already existed !\nAre you sure to overwrite ?" % folder_name):
                raise SystemError
    else:
        try:
            os.mkdir(folder_path + '/' + folder_name)
        except:
            ui_send_error("Invalid folder name: %s\nMust not contain space and \\ / : * ? \" < > |" % folder_name)
            raise IOError


# Set the color of ui items such as pushbutton -----------------------------------------------------------------------------
def ui_set_color_red(ui_obj):
    ui_obj.setStyleSheet("background-color: #FF0000")

def ui_set_color_green(ui_obj):
    ui_obj.setStyleSheet("background-color: #00FF00")

def ui_set_color_orange(ui_obj):
    ui_obj.setStyleSheet("background-color: #FF8000")

def ui_set_color_gray(ui_obj):
    ui_obj.setStyleSheet("background-color: #E1E1E1")

# UI message box -----------------------------------------------------------------------------------------------------------
def ui_send_question(msg):
    box = QMessageBox()
    box.setWindowTitle("Question")
    box.setIcon(QMessageBox.Icon.Warning)
    box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
    box.setStandardButtons(QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
    box.setText(msg)
    return box.exec()==QMessageBox.StandardButton.Yes

def ui_send_error(msg):
    box = QMessageBox()
    box.setWindowTitle("Critical")
    box.setIcon(QMessageBox.Icon.Warning)
    box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
    box.setStandardButtons(QMessageBox.StandardButton.Yes)
    box.setText(msg)
    box.exec()

def ui_send_warning(msg):
    box = QMessageBox()
    box.setWindowTitle("Warning")
    box.setIcon(QMessageBox.Icon.Warning)
    box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
    box.setStandardButtons(QMessageBox.StandardButton.Yes)
    box.setText(msg)
    box.exec()

def ui_send_info(msg):
    box = QMessageBox()
    box.setWindowTitle("Information")
    box.setIcon(QMessageBox.Icon.Information)
    box.setStyleSheet('QMessageBox {font: 9pt "Consolas"}')
    box.setStandardButtons(QMessageBox.StandardButton.Yes)
    box.setText(msg)
    box.exec()

def ui_get_password():
    (text, ok) = QInputDialog.getText(None, "Authorization", "Password:", QLineEdit.Password)
    return text if ok else None
