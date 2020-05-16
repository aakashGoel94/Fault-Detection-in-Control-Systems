# The following program has been written as an alternative to the 
# currently available solution for the 'Two Tank System' lab. 

# All instructions are for Windows Systems unless otherwise specified. 
# The instructions may work with other systems in some cases.

# We have used the GEKKO python package which is an extension of
# the APMonitor Optimization Suite.

# We make use of pyfirmata which is a python interface for the firmata protocol.
# The firmata protocol allows for communication with microcontrollers from a computer.
# The steps to setup the arduino for use with pyfirmata are as follows->
#     1) Connect the arduino to our computer.
#     2) Open the Arduino software. Go to File > Examples > Firmata. Select StandardFirmata from the list.
#     3) Verify and Upload. You can now use the pyfirmata python library to communicate with your arduino.
# In this implementation, the code is executed and run on the computer. The communication between the computer 
# and arduino is the domain of pyfirmata. Hence, the arduino must always be connected to the computer to keep 
# the whole system working like clockwork.

# Beal, L.D.R., Hill, D., Martin, R.A., and Hedengren, J. D., GEKKO Optimization Suite, 
# Processes, Volume 6, Number 8, 2018, doi: 10.3390/pr6080106. 


import pyfirmata
from pyfirmata import Arduino, util
import time
from decimal import Decimal
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import csv
from gekko import GEKKO
 

#COM port is assigned by the OS automatically.
#It can be viewed, in Windows Systems, by going to Device Manager 
#In the list there will be an option called Ports(COM and LPT)
port ='COM7' 
board = Arduino('COM7')
 
it = util.Iterator(board)  #prevents overflow of serial port
it.start()

#Setting the Input and Output pins
board.analog[0].mode = pyfirmata.INPUT
board.analog[1].mode = pyfirmata.INPUT
motor = board.get_pin('d:6:p')

m = GEKKO()
m.time = [0,1,2,4,8,12,16,20]  #time intervals on which functions need to be discretized.
 
Kp_h1 = 1.3
tau_h1 = 10.4
Kp_h2 = 1
tau_h2 = 14.4
 
p = m.MV(value=0,lb=0,ub=1)  #variable to be modified based on reading of sensors
p.STATUS = 1
p.DCOST = 0.01
p.FSTATUS = 0
 
h1 =m.Var(value=board.analog[1].read())  #Simple variable with certain constraints
 
h2 =m.CV(value=board.analog[0].read())  #variable to be controlled
h2.STATUS = 1
h2.FSTATUS = 1
h2.TAU = 20
h2.TR_INIT = 1

#Equations that are to be used by the GEKKO suite
m.Equation(tau_h1*h1.dt()==-h1 + Kp_h1*p)
m.Equation(tau_h2*h2.dt()==-h2 + Kp_h2*h1)

#Set the type of of problem and the error model type for controlled variable
m.options.IMODE = 6  #6 means we want Model Predictive Control
m.options.CV_TYPE = 2  #2 means squared error from reference trajectory

#function modelling the real world system 
def tank(levels,t,pump):
            levels[0] = board.analog[1].read()  #taking input from pin 1 for h1
            levels[1] = board.analog[0].read()  #taking input from pin 0 for h2
            h1 = max(0.0,levels[0])
            h2 = max(0.0,levels[1])
            K1 = 1.5/67.8347
            K2 = 1.5/69.6996
            Kc = 0.87
            Kp = 0.004
            #Equations that uses real time value of h1 and h2
            dhdt1 = Kp*pump - K1*h1 
            dhdt2 = K1*Kc*h1 - K2*h2
            #constraints to prevent overflow of tanks
            if h1>=0.6 and dhdt1>0.0:
            dhdt1 = 0
            if h2>=0.6 and dhdt2>0.0:
            dhdt2 = 0
            dhdt = [dhdt1,dhdt2]
            return dhdt
 
h0 = [0,0]
tf = 500
t = np.linspace(0,tf,tf+1)
sp = np.zeros(tf+1)
sp[0:] = 0.5
pump = np.zeros(tf+1)
y = np.zeros((tf+1,2))
y[0,:] = h0
plt.figure(figsize=(10,7))
plt.ion()
plt.show()
 
for i in range(1,tf):
            h2.MEAS = y[i,1]  #set initial value of h2
            h2.SP = sp[i]  #set the target value of h2
            m.solve(disp=False)  #solves the first set of equations defined 
            pump[i] = p.NEWVAL  #sets the new value to be sent to the pump
            dval=Decimal("%.2f"  % p.NEWVAL)
            motor.write(dval)  #write new value to pump
            inputs = (pump[i], )
            h = odeint(tank,h0,[0,1],inputs)  #solve the second set of equations using scipy
            #updates the initial values of y and h0 with the latest calculated value using odeint
            y[i+1,:] = h[-1,:]
            h0 = h[-1,:]
            #loop to display the graphs
            if (i%1==0):
            plt.clf()
            plt.subplot(2,1,1)
            plt.plot(t[0:i],sp[0:i],'r-')    
            plt.plot(t[0:i],y[0:i,0],'b--')
            plt.plot(t[0:i],y[0:i,1],'g-.')
            plt.ylabel('Height (m)')
            plt.legend(['Set point','Tank 1','Tank 2'])
            plt.subplot(2,1,2)
            plt.plot(t[0:i],pump[0:i],'v--')
            plt.ylabel('Motor')    
            plt.xlabel('Time (sec)')
            plt.draw()
            plt.pause(0.01)
 
data = np.vstack((t,pump))
data = np.hstack((np.transpose(data),y))
np.savetxt('data.txt',data,delimiter=',')
