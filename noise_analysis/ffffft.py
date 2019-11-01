# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import pylab as pl
from scipy import signal

ECG = np.loadtxt(r'C:\Users\ziqia\Documents\ecg\yh.dat')
fs=1000
gain=500
data = ECG[:,1]
t = ECG[:,0]

time=np.linspace(0, len(ECG)/fs*1000, num=len(data))

step = (+4.096-(-4.096))/2**10
ymv=(data-2**9)*step*1000/500

f1=45/fs
f2=55/fs
f3 = 0.5/fs
b=2000

k2=int(f1*b)
k3=int(f2*b)
k4 = int(f3*b)

x=np.ones(b)
x[k2:k3+1]=0
x[b-k3:b-k2+1]=0
x[0:k4]=0
x[b-k4:b]=0

x=np.fft.ifft(x)
x=np.real(x)
h=np.zeros(b)
h[0:int(b/2)]=x[int(b/2):b]
h[int(b/2):b]=x[0:int(b/2)]


pre_filtered = signal.lfilter(h,1,ymv)

pl.figure(1)
pl.plot(pre_filtered)
pl.title('Pre-filtered ECG')
pl.ylabel('Amplitude')
pl.xlabel('time(ms)')
pl.grid()

Pre_filtered_ECG_fft=np.fft.fft(pre_filtered)
faxis=np.linspace(0,fs,len(Pre_filtered_ECG_fft))
k1=int(len(Pre_filtered_ECG_fft)/2)
p2=pl.figure(2)
pl.plot(faxis[0:k1],abs(Pre_filtered_ECG_fft[0:k1]))
pl.xlabel('frequency(Hz)')
pl.ylabel('Amplitude')
pl.title('FFT of Pre-filtered ECG')
pl.grid()
pl.show()

real_signal = pre_filtered[1000:len(ymv)]
template = real_signal[58020:58550]


fir_coeff = template[::-1]
det=signal.lfilter(fir_coeff,1,real_signal)
det=det**2
det = det/max(det)


det_fft=np.fft.fft(det)
faxis=np.linspace(0,fs,len(det_fft))
k5=int(len(det_fft)/2)
det_fft[0:1]=0

det1=np.fft.ifft(det_fft)
det1=np.real(det1)




pk = []
count = 0

for j in range(len(det)):
    if count == 0:
        if det[j]>0.2:
            pk.append(j/1000)
            count = 300
    else:
        count = count - 1
        
        
real_bpm = np.zeros(len(pk))
for l in range(len(pk)-1):
    real_bpm[l] = 60/(pk[l+1] - pk[l])

real_bpm = real_bpm[1:len(pk)-1]
peak=pk[1:len(pk)-1]

p3 = pl.figure(3)
pl.plot(peak, real_bpm)
pl.title('heart rate')
pl.ylabel('amplitude')
pl.xlabel('time(s)')
pl.grid()
pl.show()




#p4=pl.figure(4)
#pl.plot(faxis[0:k5],abs(det_fft[0:k5]))
#pl.xlabel('frequency(Hz)')
#pl.ylabel('Amplitude')
#pl.title('FFT of square the signal')
#pl.grid()
#pl.show()
