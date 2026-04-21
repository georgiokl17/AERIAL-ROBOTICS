#
import os
import time
import numpy as np
import matplotlib.pyplot as plt
# defining parameters
deltaT=1e-3 #time 
m=1.28 #mass
ixx=0.015 
iyy=0.015
izz=0.007
J=np.array([[ixx,0,0], #inertia matrix
            [0,iyy,0],
            [0,0,izz]])
cf=6.5e-4
ct=1e-5
g=9.81
G=np.array([[1,0,0,0,0,0], #assuming rotation will stay the same since it is only 
            [0,1,0,0,0,0], #a translational simulation for now
            [0,0,1,0,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]])
w_R_b=G[0:3,0:3] #rotation matrix
T_end=15 #end simulation time
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
u_lambda2=(m*g+10)/(4*cf) #input for weight+1N 
#calculating point p
p=np.array([[L,0,0],
            [0,L,0],
            [-L,0,0],
            [0,-L,0]]) 
#calculating wrenches
for i in range(1, 5):
    f_u1= cf*v*u_lambda1 #force for weight
    tau_u1= ct*v*u_lambda1+np.cross(p[i-1],cf*v*u_lambda1) if i%2==0 else -ct*v*u_lambda1+np.cross(p[i-1],cf*v*u_lambda1) #torque for weight
    w_i_1 = np.hstack((f_u1, tau_u1))
    w_u1=w_u1+w_i_1 #summing wrench
    f_u2= cf*v*u_lambda2 #force for weight+1N
    tau_u2= ct*v*u_lambda2+np.cross(p[i-1],cf*v*u_lambda2) if i%2==0 else -ct*v*u_lambda2+np.cross(p[i-1],cf*v*u_lambda2) #torque for weight +1N
    w_i_2 = np.hstack((f_u2, tau_u2))
    w_u2=w_u2+w_i_2
    
#defining hamilton mult for quaternons
def quaternon_mult(a,b):
   a1=a[0]
   a2=b[0]
   b1=a[1]
   b2=b[1]
   c1=a[2]
   c2=b[2]
   d1=a[3]
   d2=b[3]
   w=a1*a2-b1*b2-c1*c2-d1*d2
   x=a1*b2+b1*a2+c1*d2-d1*c2
   y=a1*c2-b1*d2+c1*a2+d1*b2
   z=a1*d2+b1*c2-c1*b2+d1*a2
   q=np.array([w,x,y,z])
   return q

#defining normalizing quaternon

def unit_quaternon(a):
   norm=np.linalg.norm(a)
   w=a[0]
   x=a[1]
   y=a[2]
   z=a[3]
   q=np.array([w/norm,x/norm,y/norm,z/norm])
   return q


   
# Function that make the quadrotor follow the given trajectory
def dynamics(x0,w):
  x=x0 #initializing state
  current_time=0 #time counter
  t=[current_time]  
  pos_x=[0] #x y and z position, we need to add more but for now just for testing its good enough
  pos_y=[0]
  pos_z=[0]
  #defining our function
  def f(x,w):
    pos=x[0:3] #position
    orient=x[3:7] #quaternon orientation
    vel=x[7:10] #linear velocity
    om=x[10:13] #angular velocity with respect to the drone
    om_W=w_R_b@om #angular velocity with respect to the world
    qomg_W=np.hstack((0, om_W)) #for quaternon multiplication for xdot
    A=np.array([[1/m,0,0,0,0,0], 
                [0,1/m,0,0,0,0],
                [0,0,1/m,0,0,0],
                [0,0,0,1/ixx,0,0],
                [0,0,0,0,1/iyy,0],
                [0,0,0,0,0,1/izz]])
    sp=np.cross(om,(J@om)) #second part of vector
    B=-np.array([0,0,m*g,sp[0],sp[1],sp[2]])+G@w 
    fdot=A@B #getting second half of xdot
    orient_dot=0.5*quaternon_mult(qomg_W,orient) #quaternon mult
   # orient_dot_unit=unit_quaternon(orient_dot) #unit quaternon 
    xdot=np.hstack((vel,orient_dot,fdot)) #putting everyhting together
    return xdot
  while current_time < T_end:
    k1=f(x,w)
    k2=f(x+(deltaT/2)*k1,w)
    k3=f(x+(deltaT/2)*k2,w)
    k4=f(x+(deltaT)*k3,w) 
    xnext=x+(deltaT/6)*(k1+k2*2+k3*2+k4) #rk4 integrator
    q_orient=unit_quaternon(xnext[3:7]) #making sure quaternon is unitary
    x=np.hstack((xnext[0:3],q_orient,xnext[7:10],xnext[10:13])) #getting xk+1
    pos_x.append(xnext[0])
    pos_y.append(xnext[1])
    pos_z.append(xnext[2])
    current_time=current_time+deltaT
    t.append(current_time) #all these vectors for plotting


  fig,ax = plt.subplots(1,3) #plotting
  ax[0].plot(t, pos_x, color='red', label='x_pos')
  ax[1].plot(t, pos_y, color='green', label='y_pos')
  ax[2].plot(t, pos_z, color='blue',label='z_pos')    

  plt.show()
print(w_u1)
dynamics(x0,w_u2)