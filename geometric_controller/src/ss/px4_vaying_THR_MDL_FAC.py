# -*- coding: utf-8 -*-
"""
Created on Tue Sep 17 11:34:56 2019
px4 thrust to PWM mapping test
@author: ind
"""
import numpy as np 
import matplotlib.pyplot as plt

pwm = np.linspace(1200,2000,11)
factor = np.linspace(0,0.5,2)

plt.figure('thurst to pwm mapping')
thrust = []
for i in factor: 
    thrust = [(1-factor)*j + factor * j**2 for j in pwm]
    thrust = [i/thrust[-1] for i in thrust]
    print factor
    plt.plot(pwm, thrust, label = '{}'.format(i))

#plt.legend(loc = 3, ncol = 5)    
plt.show()
