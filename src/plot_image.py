import numpy as np
import matplotlib.pyplot as plt
import pandas as pd # Conventional alias
import os

# os.chdir("../logs/01_Quadrotor")

mylog = pd.read_csv(
    '../logs/01_Quadrotor/pom.log',
    delimiter=r'\s+',
    engine='python',
    na_values=['-', 'nan'],
    comment='#'
)

# print(mylog['x'])

time = mylog['ts']-mylog['ts'].iloc[0]
fig,ax = plt.subplots(3,2)
ax[0,0].plot(time, mylog['x'], color='purple', label='x_pos')
ax[0,0].plot(time, mylog['y'], color='green', label='y_pos')
ax[0,0].plot(time, mylog['z'], color='yellow',label='z_pos')
ax[0,0].set_xlabel('time')
ax[0,0].legend()
ax[0,0].grid()
ax[0,1].plot(time, mylog['roll'], color='purple', label='roll')
ax[0,1].plot(time, mylog['pitch'], color='green', label='pitch')
ax[0,1].plot(time, mylog['yaw'], color='yellow',label='yaw')
ax[0,1].set_xlabel('time')
ax[0,1].legend()
ax[0,1].grid()

plt.show()