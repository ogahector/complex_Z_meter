import struct
import numpy as np
import matplotlib.pyplot as plt
import os

cwd = os.getcwd()

def binary_file_write(file, data_list):
    fw = open(file, 'wb')
    for data in data_list[:-1]:
        # fw.write(struct.pack('>H', data))  # 1023 -> b'\x03\xff'
        temp = bytearray(str(data), 'utf8')
        fw.write(temp)
        fw.write(b', ')
    fw.close()


def main() -> None:
    data = np.linspace(0, 1, num=50)

    binary_file_write(cwd + "\\test.txt", data)

if __name__ == '__main__':
    main()