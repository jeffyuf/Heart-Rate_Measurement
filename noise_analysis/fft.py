# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import pylab as pl
import aaa as FIR


ECG = np.loadtxt(r'E:\temp\test.dat')
fs = 1000
gain = 500
data = ECG[:, 1]
t = ECG[:, 0]
time = np.linspace(0, len(ECG) / fs * 1000, num=len(data))

step = (+4.096 - (-4.096)) / 2 ** 10
ymv = (data - 2 ** 9) * step * 1000 / 500

p1 = pl.figure(1)
pl.plot(time, ymv)
pl.xlabel('time(ms)')
pl.ylabel('Amp')
pl.title('Original heart rate')
pl.grid()

ymv_fft = np.fft.fft(ymv)
faxis = np.linspace(0, fs, len(ymv_fft))
k1 = int(len(ymv_fft) / 2)
p2 = pl.figure(2)
pl.plot(faxis[0:k1], abs(ymv_fft[0:k1]))
pl.xlabel('frequency(Hz)')
pl.ylabel('Amplitude')
pl.title('FFT of Original heart rate')
pl.show()
pl.grid()


f1 = 45 / fs
f2 = 55 / fs
# M defines taps
M = np.arange(-200, 200 + 1)
h = 1 / (np.pi * M) * (np.sin(2 * np.pi * M * f1) - np.sin(2 * np.pi * M * f2))
h[200] = 1 - (f2 * 2 * np.pi - f1 * 2 * np.pi) / np.pi

h = h * np.hamming(401)

F1 = FIR.FIR_filter(h)

y = np.zeros(len(ymv))
for i in range(len(ymv)):
    v = np.real(ymv[i])  # put ymv in v
    y[i] = F1.filter(v)  # implment FIR filter and put result in y
