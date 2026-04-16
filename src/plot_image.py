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
#configuring time
time = mylog['ts']-mylog['ts'].iloc[0]
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



plt.show()