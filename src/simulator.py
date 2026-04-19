#
import os
import time
import numpy as np
#this is my comment ~georgio
# this connects to components running on the same host (localhost)
# to instead control components running on the remote computer "hostname" use
# g = genomix.connect('hostname')

# adapt path to your setup

# load components clients
# --- setup ----------------------------------------------------------------
#
# defining parameters
deltaT=1e-3
m=1.28
ixx=0.015
iyy=0.015
izz=0.007
cf=6.5e-4
ct=1e-5
g=9.81
G=np.array([[1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]])





def setup():
  wtvr=[]






# Function that make the quadrotor follow the given trajectory
def move():
    
    

# --- start ----------------------------------------------------------------
#
# Spin the motors and servo on current position. To be called interactively
# I have changed the folder to file path to save the log files
# The below has been changed to save
def start():
  
  move()



# --- stop -----------------------------------------------------------------
#
# Stop motors. To be called interactively
def stop():
   


## interactively, one can start the simulation with
# setup()
# start()
## and then for instance set a desired position with
# nhfc.set_position(0, 0, 1, 0)
## to stop, use
# stop


def simulation():
  setup()
  start()
  stop()
  