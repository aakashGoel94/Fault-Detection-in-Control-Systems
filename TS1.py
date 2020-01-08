# Import
from apm import *
import numpy as np
import time as tm
from scipy.integrate import odeint
import random

# Select server
s = 'http://byu.apmonitor.com'

##################################
# set up simulator
##################################
# either use Python version (False) or APM Python version (True)
sim_apm = True

if sim_apm:
    # define tank model as application
    a = 'simulate'
    # Clear previous application
    apm(s,a,'clear all')
    # Load model file
    apm_load(s,a,'gravitytank.apm')
    # Load time points for future predictions
    csv_load(s,a,'gravitytank.csv')
    #Parameters
    apm_info(s,a,'FV','pump');
    apm_info(s,a,'FV','valve');
    apm_info(s,a,'SV','h1');
    apm_info(s,a,'SV','h2');
    # imode (1=ss, 2=mpu, 3=rto, 4=sim, 5=mhe, 6=nlc)
    apm_option(s,a,'nlc.solver',3)
    # need to solve with imode 6 because of slack variables
    apm_option(s,a,'nlc.imode',6)
    apm_meas(s,a,'pump',0.1)
    output = apm(s,a,'solve')
else:
    # define tank model as function
    def tank(levels,t,pump,valve):
        h1 = levels[0]
        h2 = levels[1]
        c1 = 0.08 # inlet valve coefficient
        c2 = 0.04 # tank outlet coefficient
        dhdt1 = c1 * (1.0-valve) * pump - c2 * np.sqrt(h1)
        dhdt2 = c1 * valve * pump + c2 * np.sqrt(h1) - c2 * np.sqrt(h2)
        if h1>=1.0 and dhdt1>0.0:
            dhdt1 = 0
        if h2>=1.0 and dhdt2>0.0:
            dhdt2 = 0
            dhdt = [dhdt1,dhdt2]
            return dhdt
        # Initial conditions (levels)
        h0 = [0,0]

##################################  
# set up linear MPC
##################################
c = 'mpc'
# Clear previous application
apm(s,c,'clear all');

# load model variables and equations
apm_load(s,c,'mpc.apm');

# load data
csv_load(s,c,'mpc.csv');

# APM Variable Classification
# class = FV, MV, SV, CV
#   F or FV = Fixed value - parameter may change to a new value every cycle
#   M or MV = Manipulated variable - independent variable over time horizon
#   S or SV = State variable - model variable for viewing
#   C or CV = Controlled variable - model variable for control
apm_info(s,c,'MV','pump');
apm_info(s,c,'CV','h2');

# Options
apm_option(s,c,'nlc.imode',6);
apm_option(s,c,'nlc.solver',3);
apm_option(s,c,'nlc.nodes',3);
apm_option(s,c,'nlc.web_plot_freq',1);
apm_option(s,c,'nlc.hist_hor',70);
# online plot x-axis in seconds
apm_option(s,c,'nlc.ctrl_units',1);
apm_option(s,c,'nlc.hist_units',1);
# Bounds
apm_option(s,c,'pump.lower',0);
apm_option(s,c,'pump.upper',1);
# Turn on parameters to control
apm_option(s,c,'pump.status',1);
apm_option(s,c,'pump.dmax',.1);
apm_option(s,c,'pump.fstatus',0);

apm_option(s,c,'h2.status',1);
apm_option(s,c,'h2.fstatus',1);
apm_option(s,c,'h2.tau',2);
apm_option(s,c,'h2.tr_init',0);

apm_option(s,c,'h2.wmeas',100);
apm_option(s,c,'h2.wmodel',1);
h2sp = 0.5
apm_option(s,c,'h2.sphi',h2sp+.01)
apm_option(s,c,'h2.splo',h2sp-.01)

# initialize values
time = 0
dt = 1
h2 = 0

h1_tracker = np.array([])
h2_tracker = np.array([])
sp_tracker = np.array([])
pump_tracker = np.array([])
valve_tracker = np.array([])
time_tracker = np.array([])

for isim in range(201):

    # MPC Controller #######################
    # Change temperature set point
    if isim == 100:
        h2sp = 0.8
        apm_option(s,c,'h2.sphi',h2sp+.01)
        apm_option(s,c,'h2.splo',h2sp-.01)
    # Input height measurement
    apm_meas(s,c,'h2',h2)
    # Solve
    output = apm(s,c,'solve');
    d = apm_sol(s,c)
    # Output MV new value, check if good solution
    if (apm_tag(s,c,'nlc.appstatus')==1):
        pump = apm_tag(s,c,'pump.newval');
    else:
        pump = 0
        print(solver_output)
        print("Warning: Solver Failed to Converge for Controller")
    #########################################


    # 2 Tank Simulator ######################
    # Valve Disturbance
    valve = random.random()*0.2 #(from 0 to .2)
    if sim_apm:
        # APM Python Simulator
        # Insert pump
        apm_meas(s,a,'pump',pump)
        # Insert valve disturbance
        apm_meas(s,a,'valve',valve)
        # Run on server
        solver_output = apm(s,a,'solve')
        # Read levels
        h1 = apm_tag(s,a,'h1.model')
        h2 = apm_tag(s,a,'h2.model')
    else:
        # Python Simulator
        # Specify the pump and valve
        inputs = (pump,valve)
        # Integrate the model
        h = odeint(tank,h0,[0,1],inputs)
        # Reset the initial condition
        h0 = h[-1,:]
        # Simulator output
        h1 = h0[0]
        h2 = h0[1]
        
    #####################################

    # Print data
    if np.mod(isim,20)==0:
        print('Time      Pump     Height 1    Height 2')
    print('{0:2f} {1:2f} {2:2f} {3:2f}'.format(time,pump,h1,h2))

    # Increment time
    time = time + dt

    if (isim==1):
        # Open Web Viewers
        url = apm_web(s,c)

    sp_tracker = np.append(sp_tracker,h2sp)
    h1_tracker = np.append(h1_tracker,h1)
    h2_tracker = np.append(h2_tracker,h2)
    pump_tracker = np.append(pump_tracker,pump)
    valve_tracker = np.append(valve_tracker,valve)
    time_tracker = np.append(time_tracker,isim)

import matplotlib.pyplot as plt

plt.subplot(211)
plt.plot(time_tracker,h1_tracker,'-r')
plt.plot(time_tracker,h2_tracker,'-b')
plt.plot(time_tracker,sp_tracker,'-k')
plt.legend(['$height_1$','$height_2$','$target$'],loc=4)
plt.ylabel('height ')
plt.subplot(212)
plt.xlabel('time (s)')
plt.ylabel('pump')
plt.plot(time_tracker,pump_tracker,'-r')
plt.plot(time_tracker,valve_tracker,'.b')
plt.legend(['$pump$','$valve$'],loc=2)
plt.show()
