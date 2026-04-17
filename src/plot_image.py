import numpy as np
import matplotlib.pyplot as plt
import pandas as pd # Conventional alias
import os
#from quadrotor_under_contropl import geom 
#the above import is not working
# os.chdir("../logs/01_Quadrotor")

mylog = pd.read_csv(
    '../logs/01_Quadrotor/pom.log',
    delimiter=r'\s+',
    engine='python',
    na_values=['-', 'nan'],
    comment='#'
)

# print(mylog['x'])
#configuring time
time = mylog['ts']-mylog['ts'].iloc[0]
#first figure for position and velocity and acc
fig,ax = plt.subplots(3,2)

#subplot of x y z values
ax[0,0].set_ylim(-2,2)
ax[0,0].plot(time, mylog['x'], color='red', label='x_pos')
ax[0,0].plot(time, mylog['y'], color='green', label='y_pos')
ax[0,0].plot(time, mylog['z'], color='blue',label='z_pos')
ax[0,0].set_xlabel('time')
ax[0,0].legend()
ax[0,0].grid()

#subplot of roll pitch yaw values
ax[0,1].set_ylim(-4,4)
ax[0,1].plot(time, mylog['roll'], color='red', label='roll')
ax[0,1].plot(time, mylog['pitch'], color='green', label='pitch')
ax[0,1].plot(time, mylog['yaw'], color='blue',label='yaw')
ax[0,1].set_xlabel('time')
ax[0,1].legend()
ax[0,1].grid()

#subplot of vx vy vz values
ax[1,0].set_ylim(-2,2)
ax[1,0].plot(time, mylog['vx'], color='red', label='x_vel')
ax[1,0].plot(time, mylog['vy'], color='green', label='y_vel')
ax[1,0].plot(time, mylog['vz'], color='blue',label='z_vel')
ax[1,0].set_xlabel('time')
ax[1,0].legend()
ax[1,0].grid()

#subplot of wx wy wz values
ax[1,1].set_ylim(-3,3)
ax[1,1].plot(time, mylog['wx'], color='red', label='wx')
ax[1,1].plot(time, mylog['wy'], color='green', label='wy')
ax[1,1].plot(time, mylog['wz'], color='blue',label='wz')
ax[1,1].set_xlabel('time')
ax[1,1].legend()
ax[1,1].grid()

#subplot of ax ay az values
ax[2,0].set_ylim(-4,4)
ax[2,0].plot(time, mylog['ax'], color='red', label='x_acc')
ax[2,0].plot(time, mylog['ay'], color='green', label='y_acc')
ax[2,0].plot(time, mylog['az'], color='blue',label='z_acc')
ax[2,0].set_xlabel('time')
ax[2,0].legend()
ax[2,0].grid()

#for errors
mynhfclog = pd.read_csv(
    '../logs/01_Quadrotor/nhfc.log',
    delimiter=r'\s+',
    engine='python',
    na_values=['-', 'nan'],
    comment='#'
)
time2 = mynhfclog['ts']-mylog['ts'].iloc[0]
fig,ax = plt.subplots(2,2)

#subplot of positions errors 
ax[0,0].set_ylim(-2,2)
ax[0,0].plot(time2, mynhfclog['e_x'], color='red', label='x_err')
ax[0,0].plot(time2, mynhfclog['e_y'], color='green', label='y_err')
ax[0,0].plot(time2, mynhfclog['e_z'], color='blue',label='z_err')
ax[0,0].set_xlabel('time')
ax[0,0].legend()
ax[0,0].grid()

#subplot of roll pitch yaw errors
ax[0,1].set_ylim(-4,4)
ax[0,1].plot(time2, mynhfclog['e_rx'], color='red', label='roll_err')
ax[0,1].plot(time2, mynhfclog['e_ry'], color='green', label='pitch_err')
ax[0,1].plot(time2, mynhfclog['e_rz'], color='blue',label='yaw_err')
ax[0,1].set_xlabel('time')
ax[0,1].legend()
ax[0,1].grid()

#subplot of vx vy vz errors
ax[1,0].set_ylim(-1,1)
ax[1,0].plot(time2, mynhfclog['e_vx'], color='red', label='x_vel_err')
ax[1,0].plot(time2, mynhfclog['e_vy'], color='green', label='y_vel_err')
ax[1,0].plot(time2, mynhfclog['e_vz'], color='blue',label='z_vel_err')
ax[1,0].set_xlabel('time')
ax[1,0].legend()
ax[1,0].grid()

#subplot of wx wy wz errors
ax[1,1].set_ylim(-4,4)
ax[1,1].plot(time2, mynhfclog['e_wx'], color='red', label='wx_err')
ax[1,1].plot(time2, mynhfclog['e_wy'], color='green', label='wy_err')
ax[1,1].plot(time2, mynhfclog['e_wz'], color='blue',label='wz_err')
ax[1,1].set_xlabel('time')
ax[1,1].legend()
ax[1,1].grid()

# For forces
myrotolog = pd.read_csv(
    '../logs/01_Quadrotor/rotorcraft.log',
    delimiter=r'\s+',
    engine='python',
    na_values=['-', 'nan'],
    comment='#'
)

# Replacing the NaN of ts
time3 = myrotolog['ts'].fillna(0)
time3 = time3-myrotolog['ts'].iloc[0]

cf =0.00064 # From SDF file of quadrotor

# Replacing the NaN of meas_v
omega0 = myrotolog['meas_v0'].fillna(0)
omega1 = myrotolog['meas_v1'].fillna(0)
omega2 = myrotolog['meas_v2'].fillna(0)
omega3 = myrotolog['meas_v3'].fillna(0)

# Computation of velocities in rad/s
omega0 = omega0*2*np.pi
omega1 = omega1*2*np.pi
omega2 = omega1*2*np.pi
omega3 = omega3*2*np.pi

# Computation of forces
force1 = cf*omega0**2 
force2 = cf*omega1**2
force3 = cf*omega2**2
force4 = cf*omega3**2


# Forces plotting
fig, ax = plt.subplots(2,2)
ax[0,0].plot(time3, force1, color='red', label='Force 1')
ax[0,0].set_xlabel('time')
ax[0,0].legend()
ax[0,0].grid()
ax[0,1].plot(time3, force2, color='green', label='Force 2')
ax[0,1].set_xlabel('time')
ax[0,1].legend()
ax[0,1].grid()
ax[1,0].plot(time3, force3, color='blue', label='Force 3')
ax[1,0].set_xlabel('time')
ax[1,0].legend()
ax[1,0].grid()
ax[1,1].plot(time3, force4, color='orange', label='Force 4')
ax[1,1].set_xlabel('time')
ax[1,1].legend()
ax[1,1].grid()

plt.show()