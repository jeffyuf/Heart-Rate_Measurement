# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import pylab as pl
import ass22 as FIR


ECG = np.loadtxt('G:\old\ecg')
fs=1000
gain=500
data = ECG[:,1]
t = ECG[:,0]
time=np.linspace(0, len(ECG)/fs*1000, num=len(data))


step = (+4.096-(-4.096))/2**12
ymv=(data-2**11)*step*1000/500

p1=pl.figure()
pl.plot(time,ymv)
pl.xlabel('time(ms)')
pl.ylabel('ECG/RAW(mv)')
pl.title('Original ECG')
pl.grid()


ymv_fft=np.fft.fft(ymv)
faxis=np.linspace(0,fs,len(ymv_fft))
k1=int(len(ymv_fft)/2)
p2=pl.figure()
pl.plot(faxis[0:k1],abs(ymv_fft[0:k1]))
pl.xlabel('frequency(Hz)')
pl.ylabel('Amplitude')
pl.title('FFT of ECG')
pl.show()
pl.grid()

#Q3
f1=45/fs
f2=55/fs
#M defines taps
M=np.arange(-200,200+1)
h=1/(np.pi*M)*(np.sin(2*np.pi*M*f1)-np.sin(2*np.pi*M*f2))
h[200]=1-(f2*2*np.pi-f1*2*np.pi)/np.pi

h=h*np.hamming(401)

F1=FIR.FIR_filter(h)

y=np.zeros(len(ymv))
for i in range(len(ymv)):
    v=np.real(ymv[i])    #put ymv in v
    y[i]=F1.filter(v)     #implment FIR filter and put result in y    


p3=pl.figure()
pl.title('ECG after 50Hz notch filter')
pl.xlabel('time(ms)')
pl.ylabel('ECG(mv)')
pl.plot(time,y)
pl.grid()

y_fft=np.fft.fft(y)
faxis=np.linspace(0,fs,len(y_fft))
k5=int(len(y_fft)/2)
p4=pl.figure()
pl.plot(faxis[0:k5],abs(y_fft[0:k5]))
pl.xlabel('frequency(Hz)')
pl.ylabel('Amplitude')
pl.title('FFT of ECG after 50Hz notch filter')
pl.show()
pl.grid()

#Q4 Remove the baaseline shift and 50Hz with DFT

f3 = 3/fs #remove baseline shift, choose 3Hz so that int(k4) round to 1
b=len(M)
k2=int(f1*b)  #f1 shown before, f1=45/fs 
k3=int(f2*b)  #f2 shown before, f2=55/fs
k4=int(f3*b)

x=np.ones(b-1)
x[k2:k3+1]=0
x[b-k3:b-k2+1]=0
x[0:k4]=0
x[b-k4:b]=0

x=np.fft.ifft(x)
x=np.real(x)
h=np.zeros(b-1)
h[0:int(b/2)]=x[int(b/2):b]
h[int(b/2):b]=x[0:int(b/2)]

F2=FIR.FIR_filter(h)

y2=np.zeros(len(ymv))
for i in range(len(ymv)):
    v=np.real(ymv[i])    
    y2[i]=F2.filter(v)     


p5=pl.figure()
pl.title('ECG after 50Hz and removal baseline shift')
pl.xlabel('time(ms)')
pl.ylabel('ECG(mv)')
pl.plot(time,y2)
pl.grid()

y2_fft=np.fft.fft(y2)
faxis=np.linspace(0,fs,len(y2_fft))
k6=int(len(y2_fft)/2)
p6=pl.figure()
pl.plot(faxis[0:k6],abs(y2_fft[0:k6]))
pl.xlabel('frequency(Hz)')
pl.ylabel('Amplitude')
pl.title('FFT of ECG after 50Hz and removal baseline shift')
pl.show()
pl.grid()










