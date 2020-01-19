import pyfirmata
from pyfirmata import Arduino, util
import time
from decimal import Decimal
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import csv
from gekko import GEKKO

port ='COM7'
board = Arduino('COM7')

it = util.Iterator(board)
it.start()

board.analog[0].mode = pyfirmata.INPUT
board.analog[1].mode = pyfirmata.INPUT
motor = board.get_pin('d:6:p')


m = GEKKO()
m.time = [0,1,2,4,8,12,16,20]

Kp_h1 = 1.3
tau_h1 = 10.4
Kp_h2 = 1
tau_h2 = 14.4

p = m.MV(value=0,lb=0,ub=1)
p.STATUS = 1
p.DCOST = 0.01
p.FSTATUS = 0

h1 =m.Var(value=board.analog[1].read())

h2 =m.CV(value=board.analog[0].read())
h2.STATUS = 1
h2.FSTATUS = 1
h2.TAU = 20
h2.TR_INIT = 1

m.Equation(tau_h1*h1.dt()==-h1 + Kp_h1*p)
m.Equation(tau_h2*h2.dt()==-h2 + Kp_h2*h1)

m.options.IMODE = 6
m.options.CV_TYPE = 2

def tank(levels,t,pump):
    levels[0] = board.analog[1].read()
    levels[1] = board.analog[0].read()
    h1 = max(0.0,levels[0])
    h2 = max(0.0,levels[1])
    K1 = 1.5/67.8347
    K2 = 1.5/69.6996
    Kc = 0.87
    Kp = 0.004
    dhdt1 = Kp*pump - K1*h1   
    dhdt2 = K1*Kc*h1 - K2*h2
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
    h2.MEAS = y[i,1]
    h2.SP = sp[i]
    m.solve(disp=False)
    pump[i] = p.NEWVAL
    dval=Decimal("%.2f"  % p.NEWVAL)
    motor.write(dval)
    inputs = (pump[i], )
    h = odeint(tank,h0,[0,1],inputs)
    y[i+1,:] = h[-1,:]
    h0 = h[-1,:]
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
