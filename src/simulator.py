#
import os
import time
import numpy as np
 
# defining parameters
deltaT=1e-3 #time 
m=1.28
ixx=0.015
iyy=0.015
izz=0.007
cf=6.5e-4
ct=1e-5
g=9.81
G=np.array([[1,0,0,0,0,0], #assuming rotation will stay the same since it is only a translational simulation for now
            [0,1,0,0,0,0],
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]])
L=0.125 #distance of rotor from center of mass of drone


# --- setup ----------------------------------------------------------------
#


x0=np.array([0,0,0, #position
            1,0,0,0, #rotation quaternon that needs to be unitary
            0,0,0, #linear velocity
            0,0,0]) #angular velocity

v=np.array([0,0,1]) #orientation of underactuated quadorotor motor
w_u1=0 #for only weight
u_lambda1=m*g/(4*cf) #input for weight
w_u2=0 #for weight + 1N
u_lambda2=(m*g+1)/(4*cf) #input for weight+1N 
#calculating point p
for j in range(1,4):
    p=np.array([[L,0,0],
                [0,L,0],
                [-L,0,0],
                [0,-L,0]]) 
#calculating wrenches
for i in range(1, 5):
    f_u1= cf*v*u_lambda1
    tau_u1= ct*v*u_lambda1+np.cross(p[i-1],cf*v*u_lambda1) if i%2==0 else -ct*v*u_lambda1+np.cross(p[i-1],cf*v*u_lambda1)
    w_i_1 = np.hstack((f_u1, tau_u1))
    w_u1=w_u1+w_i_1
    f_u2= cf*v*u_lambda2
    tau_u2= ct*v*u_lambda2+np.cross(p[i-1],cf*v*u_lambda2) if i%2==0 else -ct*v*u_lambda2+np.cross(p[i-1],cf*v*u_lambda2)
    w_i_2 = np.hstack((f_u2, tau_u2))
    w_u2=w_u2+w_i_2
    
  


# Function that make the quadrotor follow the given trajectory
def dynamics():
  x=x0  
  
  while True:
    cmd = input("write stop to end: ")
    if cmd == "stop":
        break
    else:
       pos=x[0:3]
       orient=x[3:7]
       vel=x[7:10]
       om=x[10:13]

       A1=np.array([[1/m,0,0,0,0,0],
                   [0,1/m,0,0,0,0],
                   [0,0,1/m,0,0,0],
                   [0,0,0,1/ixx,0,0],
                   [0,0,0,0,1/iyy,0],
                   [0,0,0,0,0,1/izz]])
       sp=np.cross(om,(j@om))
       B1=np.array(-[0,0,pos[2],sp[0],sp[1],sp[2]])+G@w_i_1
       fdot1=A1@B1
       

      

    

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
  