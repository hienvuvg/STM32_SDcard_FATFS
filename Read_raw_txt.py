import datetime as dt
import re

# from bitstream import BitStream
import struct
from datetime import datetime

import matplotlib.dates as md
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def convert(filename):
    f = open(filename, "r")
    a = np.fromfile(f, dtype=np.int16, count=-1, offset=0)
    length = len(a) - 1
    print(length)
    output = np.zeros(length)

    i = 0
    j = 0
    while i < length:
        # ignore 0s at the end of each writing cycle
        if a[i] == 0 and a[i + 1] == 0:
            i += 2
            continue

        if a[i] == 31313:
            i += 1
            if a[i] == 31212:
                print("t", end=" ")
        else:
            output[j] = a[i]
            j += 1
        i += 1
    return output


import os
from os import path

input_loc = path.join(path.dirname(__file__))  # Folder

data = convert(input_loc + "/RTEST.TXT")

plt.plot(data)
# print(time_accel[:19])
# xfmt = md.DateFormatter('%m-%d %H:%M:%S.%')
# ax.xaxis.set_major_formatter(xfmt)
plt.ylim([-34000, +34000])

plt.grid()
# plt.legend()
# plt.xlabel("Time")
# plt.ylabel("Acceleration")
plt.show()
