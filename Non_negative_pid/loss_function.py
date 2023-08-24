import numpy as np
import serial
import matplotlib.pyplot as plt
import serial.tools.list_ports
import time
import xlwt
import scipy.signal as signal
import pandas as pd
import sympy
import statsmodels.tsa.api as smt
from scipy.integrate import simpson


def if_unstable(x):
    if any(num < 0 for num in x):
        return 1
    else:
        return 0

def MSELoss(x:list,y:list):
    assert len(x)==len(y)
    x=np.array(x)
    y=np.array(y)
    loss=np.sum(np.square(x-y)/len(x))
    if if_unstable(x)==1:
        loss=100000000
    return loss

def L2Loss(x:list,y:list):
    assert len(x)==len(y)
    x=np.array(x)
    y=np.array(y)
    loss=np.sqrt(np.sum(np.square(x-y)/len(x)))
    if if_unstable(x)==1:
        loss=100000000
    return loss

def L1Loss(y_true,y_pred, weights ):
    assert len(y_true)==len(y_pred)==len(weights)
    y_true=np.array(y_true)
    y_pred=np.array(y_pred)
    weights=np.array(weights)
    absolute_errors = np.abs(y_true - y_pred)
    weighted_errors = absolute_errors * weights
    loss = np.mean(weighted_errors)
    if if_unstable(y_true)==1:
        loss=100000000
    return loss

