import serial
import numpy as np
import ast
import matplotlib.pyplot as plt
import iqmethod as iq
from enum import Enum

ser = serial.Serial('COM9', 115200)
vmeas0 = []
vmeas1 = []

measurement: dict[int, complex] = {}
sc_calibration: dict[int, complex] = {}
oc_calibration: dict[int, complex] = {}

command = ""
"""
NOTE:
Possible Commands:
    1. b'M\n' : sends measurement instruction
    2. b'C\n' : sends calibration instruction
    3. b'x\n' : with x = [0:3] chooses which resistor to do the measurement with 
"""

class Resistors(Enum):
    RES0 = 0,
    RES1 = 1,
    RES2 = 2,
    RES3 = 3

ADC_SAMPLES_PER_CHANNEL = 5000

CONFIGURATIONS = [ # see Keysigh Impedance Measurement handbook for this
    "full parallel", # coils with high loss
    "c shunt", # coils in general, resistors
    "rc parallel shunt", # high value resistors
    "full series" # capacitors
    # "resonator"
]

def get_RLC_theoretical_response(R, L, C, freqs, configuration: str):
    if configuration.lower() == 'full parallel':
        return [
            1 / (1/R + 1/(1j*2*np.pi*freq*L) + 1/( -1j/(2*np.pi*freq*C) ) )
            for freq in freqs
        ]
    elif configuration.lower() == 'c shunt':
        return [
            1 / ( (1j*2*np.pi*freq*C) + 1/( R + 1j*2*np.pi*freq*L ) )
            for freq in freqs
        ]
    elif configuration.lower() == 'rc parallel shunt':
        return [
            1j*2*np.pi*freq*L + 1 / ( 1/R + 1j*2*np.pi*freq*C )
            for freq in freqs
        ]
    else: # series
        return [
            R + (1j*2*np.pi*freq*L) - (1j /(2*np.pi*freq*C) )
            for freq in freqs
        ]
        
    
def Qfactor_RLC(R, L, C, configuration: str):
    if configuration.lower() == 'parallel':
        return R * np.sqrt(C/L)
    else: # series
        return 1/R * np.sqrt(L/C)

def choose_resistor(res_num: Resistors):
    if res_num not in range(4) or not isinstance(res_num, Resistors):
        raise Exception('Bad resistor chosen')
    
    res_num = bytearray(str(int(res_num)) + '\n')

    ser.write(res_num)

def get_calibrations():
    choose_resistor(Resistors.RES0)

    


while True:
    if command == "":
        command = input("Enter command ('C' for calibration, 'M' for measurement): ")
    if command == 'C':
        ser.write(b'C\n')
        command == 'measuring'
    elif command == 'M':
        ser.write(b'M\n')
        command == 'calibrating'
    # else:
    #     continue


    reading = ser.readline()
    print(reading)
    reading = str(reading)

    if 'DONE' in reading:
        frequencies = np.array(list(measurement.keys()))
        phasors = np.array(list(measurement.values()))
        magnitudes = np.array([abs(val) for _, val in measurement.items()])
        phases = [np.atan2(np.imag(val), np.real(val)) for _, val in measurement.items()]

        magnitudes_db = 20*np.log10(magnitudes)

        f1 = plt.figure()
        plt.semilogx(frequencies, magnitudes_db)
        plt.xlabel('Frequency [Hz]')
        plt.ylabel('Magnitude [dB]')
        plt.title("Magnitude plot")
        plt.grid(True)
        f1.savefig('mag plot no buffer.png')

        f2 = plt.figure()
        plt.semilogx(frequencies, 180/np.pi*np.array(phases))
        plt.xlabel('Frequency [Hz]')
        plt.ylabel('Phase [deg]')
        plt.title("Phase plot")
        plt.grid(True)
        f2.savefig('phase plot no buffer.png')

        # calculate an IMPEDANCE based on received voltage
        Zm = 1000 * phasors / (1 - phasors)

        f3 = plt.figure()
        plt.semilogx(frequencies, 20*np.log10(abs(Zm)))
        plt.xlabel('Frequency [Hz]')
        plt.ylabel('Magnitude [dB]')
        plt.title("Magnitude plot")
        plt.grid(True)

        f4 = plt.figure()
        plt.semilogx(frequencies, 180/np.pi*np.array( np.atan2(np.imag(Zm), np.real(Zm)) ))
        plt.xlabel('Frequency [Hz]')
        plt.ylabel('Phase [deg]')
        plt.title("Phase plot")
        plt.grid(True)

        plt.show()
        break

    if 'Measurement:Freq ' in reading:
        vmeas0.clear()
        vmeas1.clear()
        reading = reading.replace('Measurement:Freq', "")
        reading = reading.replace("\\", "")
        reading = reading.replace('r', "")
        reading = reading.replace('n', "")
        reading = reading.split()
        # print(reading)
        f0 = ast.literal_eval(reading[1])
        fs = ast.literal_eval(reading[2])
        res = ast.literal_eval(reading[3])
        temp = [ast.literal_eval(r) for r in reading[4:-1]] # the rest

        # if transmitting the list of vals
        # for i in range(len(temp)):
        #     if i % 2 == 0:
        #         vmeas0.append(temp[i])
        #     else:
        #         vmeas1.append(temp[i])
        # print(f'VMEAS0: {vmeas0}')
        # print(f'\nVEAMS1: {vmeas1}')
        # phasor = iq.phasor_2sig(vmeas1, vmeas0, f0, fs)

        phasor = temp[0] * np.exp(1j*temp[1])

        measurement[f0] = phasor

        print(f'f0: {f0} | fs: {fs} | res:{res} | phasor: {phasor:.3f}')

        # plt.figure()
        # plt.plot(vmeas0)
        # plt.plot(vmeas1)
        # # plt.plot(np.array(vmeas0) - np.array(vmeas1))
        # plt.title(f'f0: {f0} | fs: {fs} | VMEAS1 Phasor: {phasor:.3f}')        
        # plt.legend(['VMEAS0', 'VMEAS1', 'VDIFF'])

        # # plt.figure()
        # # plt.plot(vmeas1, 'r')
        # # plt.title(f'f0: {f0} | fs: {fs} | VMEAS1 Phasor: {iq.phasor_1sig(vmeas1, f0, fs):.3f}')

        # plt.show()
        # break

    # print(str(reading))
    