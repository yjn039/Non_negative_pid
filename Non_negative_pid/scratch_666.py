import math

import numpy as np
import serial
import matplotlib.pyplot as plt
import serial.tools.list_ports
import time
import scipy.signal as signal
import pandas as pd
import sympy
import statsmodels.tsa.api as smt
from scipy.integrate import simpson


def np_move_avg(a,n,mode="same"): #moving average filter
    return(np.convolve(a, np.ones((n,))/n, mode=mode))


n=[1,3,5,7,9,11,10,9,10,11,10,9,10,11,10,9,10,11,10,9,10,11,10,9,10,11,10,9,10,11,10,9,10]
n=[1,3,5,7,9,11,13,15,13,11,9,11,10,10.1,10.2,10.1,9.9,10,9.8,10,10.3,10,9.8,10,9.9,10.2,10.1,10]
#n=[1,3,5,7,9,11,13,15,13,11,9,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10]
#n=[10,10.1,10.2,10.1,9.9,10,9.8,10,10.3,10,9.8,10,9.9,10.2,10.1,10,10,10.1,10.2,10.1,9.9,10,9.8,10,10.3,10,9.8,10,9.9,10.2,10.1,10]

plt.subplot(3,2,1)
plt.title('example 2')
plt.plot(n)
plt.plot(8,n[8],'*')
plt.plot(12,n[12],'*')
plt.plot(15,n[15],'*')
plt.subplot(3,2,2)
plt.title('autocorr from orange point 2 end')
acf=smt.stattools.acf(n[8:len(n)], nlags=len(n)-9)
acf_ex = acf[signal.argrelextrema(acf, np.greater)]
acf_ex_position=signal.argrelextrema(acf, np.greater)[0]
acf_min = acf[signal.argrelextrema(acf, np.less)]
acf_min_position=signal.argrelextrema(acf, np.less)[0]
plt.plot(acf_min_position,acf_min,'x')
plt.plot(acf_ex_position,acf_ex,'x')
print(sum(acf_ex))
plt.plot(acf)
plt.subplot(3,2,3)
plt.title('autocorr from green point 2 end ')
acf=smt.stattools.acf(n[12:len(n)], nlags=len(n)-13)
acf_ex = acf[signal.argrelextrema(acf, np.greater)]
acf_ex_position=signal.argrelextrema(acf, np.greater)[0]
acf_min = acf[signal.argrelextrema(acf, np.less)]
acf_min_position=signal.argrelextrema(acf, np.less)[0]
plt.plot(acf_min_position,acf_min,'x')
plt.plot(acf_ex_position,acf_ex,'x')
print(sum(acf_ex))
plt.plot(acf)
plt.subplot(3,2,4)
plt.title('autocorr from red point 2 end ')
acf=smt.stattools.acf(n[15:len(n)], nlags=len(n)-16)
acf_ex = acf[signal.argrelextrema(acf, np.greater)]
acf_ex_position=signal.argrelextrema(acf, np.greater)[0]
acf_min = acf[signal.argrelextrema(acf, np.less)]
acf_min_position=signal.argrelextrema(acf, np.less)[0]
plt.plot(acf_min_position,acf_min,'x')
plt.plot(acf_ex_position,acf_ex,'x')
print(sum(acf_ex))
plt.plot(acf)
plt.subplot(3,2,5)
sum_acf=[0]*len(n)
for i in range(0, len(n) - 1):
    acf=smt.stattools.acf(n[i:len(n)], nlags=len(n)-i-1)
    for j in range(0,len(acf)):
        if acf[j]<0:
            break

    #sensor_data_moveave = np_move_avg(acf, 10, mode='same')
    #noise=sensor_data_moveave-acf
    acf_ex = acf[signal.argrelextrema(acf, np.greater)]
    acf_positive = [i for i in acf_ex if i >= 0]
    acf_negative = [i for i in acf_ex if i < 0]
    #plt.plot(acf)
    #plt.plot(sensor_data_moveave)
    #plt.plot(noise)
    #plt.show()
    #print(acf)
    print(acf_ex)
    if len(acf_positive) > 1:
        minus = all(acf_positive[m] > acf_positive[m + 1] for m in range(0, len(acf_positive) - 1))
        if minus == True:
            sum_acf[i]=sum(acf_positive)
            print(sum_acf[i])
plt.plot(sum_acf)
plt.title('sum of the extreme point for all autocorr function')
plt.subplot(3,2,6)
plt.title('find stable point (max of sum extreme point)')
plt.plot(n)
plt.plot(np.argmax(sum_acf),n[np.argmax(sum_acf)],'*')


plt.show()


