# Copyright (c) 2026 IRISA/CNRS-INRIA
# All rights reserved.
#
# Redistribution  and  use  in  source  and binary  forms,  with  or  without
# modification, are permitted provided that the following conditions are met:
#
#   1. Redistributions of  source  code must retain the  above copyright
#      notice and this list of conditions.
#   2. Redistributions in binary form must reproduce the above copyright
#      notice and  this list of  conditions in the  documentation and/or
#      other materials provided with the distribution.
#
# THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
# WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
# MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
# ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
# WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
# IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
#                                         Gianluca Corsini on Thu Apr 16 2026

# ############################################
#  PLACE HERE YOUR NECESSARY FUNCTION IMPORTS
# ############################################

import genomix
import math
import numpy as np
import os
import time
import matplotlib.pyplot as plt
import math

# --- setup ----------------------------------------------------------------
def setup():

  # configure quadrotor geometry: 4 rotors, not tilted, 23cm arms
    geom = {
        'rotors': 4, 'cx': 0, 'cy': 0, 'cz': 0, 'armlen': 0.23, 'mass': 1.28,
        'rx': 0, 'ry': 0, 'rz': -1, 'cf': 6.5e-4, 'ct': 1e-5
    }

    nhfc.set_gtmrp_geom(geom) #changed this only to have the cf as a variable we can use
  
    #Created a port my_state_man where maneuver component will receive the state of the drone from our simulator
    my_state_port_maneuver = maneuver.state('my_state_man')
    
    # connect maneuver to state port which will be updated by our simulator
    maneuver.connect_port({ 'local': 'state', 'remote': 'my_state_man' })
    
    my_state_port_nhfc = nhfc.state('my_state_nhfc')
    
    # connect nhfc to state port which will be updated by our simulator
    nhfc.connect_port({ 'local': 'state', 'remote': 'my_state_nhfc' })
    
    # connect nhfc reference port to maneuver desired port, so that the maneuver component can send the desired trajectory to nhfc
    nhfc.connect_port({'local':'reference','remote': 'maneuver/desired'})
    
    return  my_state_port_nhfc, my_state_port_maneuver

# --- start ----------------------------------------------------------------
def start():
    #I have nhfc to set the current position
    #This is required for the maneuver component to know the initial state of the drone   
    #nhfc.set_current_position()
    #I have also started the servo of the nhfc component to make sure it is ready to receive commands from the maneuver component
    nhfc.servo(ack=True)
    
    pass 
# --- stop -----------------------------------------------------------------
def stop():
    # ########################################
    #  FILL THIS FUNCTION, THEN REMOVE 'pass'
    # ########################################
    pass

# --- state_to_system --------------------------------------------------------

#Modified this from state_to_nhfc to be able to send the state of the drone to both nhfc and maneuver components
def state_to_system(state_port, state: np.array):
    def _get_time():
        # returns a tuple of the type (<sec>,<nsec>)
        now = math.modf(time.clock_gettime(time.CLOCK_REALTIME))
        return (int(now[1]), int(now[0]*1e9))

    # https://git.openrobots.org/projects/libmrsim/repository/libmrsim/revisions/master/entry/src/sim.c#L66
    pstddev = 1e-3
    qstddev = 1e-3
    vstddev = 1e-3
    wstddev = 3e-3
    astddev = 2e-2

    """ covariances """
    # Covariances documentation at
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/t3d.idl#L41

    pos_cov = [(pstddev)**2, 0, (pstddev)**2, 0, 0, (pstddev)**2]

    att_cov = [0 for i in range(10)]
    # https://git.openrobots.org/projects/mrsim-genom3/repository/mrsim-genom3/revisions/master/entry/codels/sim.c#L52
    qw = state[3]
    qx = state[4]
    qy = state[5]
    qz = state[6]
    att_cov[0] = (qstddev**2) * (1 - qw*qw);
    att_cov[1] = (qstddev**2) * -qw*qx;
    att_cov[2] = (qstddev**2) * (1 - qx*qx);
    att_cov[3] = (qstddev**2) * qw*qy;
    att_cov[4] = (qstddev**2) * -qx*qy;
    att_cov[5] = (qstddev**2) * (1 - qy*qy);
    att_cov[6] = (qstddev**2) * -qw*qz;
    att_cov[7] = (qstddev**2) * -qx*qz;
    att_cov[8] = (qstddev**2) * -qy*qz;
    att_cov[9] = (qstddev**2) * (1 - qz*qz);

    att_pos_cov = [0 for i in range(4*3)]

    vel_cov = [(vstddev)**2, 0, (vstddev)**2, 0, 0, (vstddev)**2]
    avel_cov = [(wstddev)**2, 0, (wstddev)**2, 0, 0, (wstddev)**2]
    acc_cov = [(astddev)**2, 0, (astddev)**2, 0, 0, (astddev)**2]

    aacc_cov = [0 for i in range(6)]

    now = _get_time()

    # port and message descriptions at
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/pose_estimator.gen
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/t3d.idl
    data = { "state": {
            "ts" : {"sec": now[0], "nsec": now[1]},
            "intrinsic": False,
            "pos": ({"x": state[0], "y": state[1], "z": state[2]}),
            "att": ({"qw": state[3], "qx": state[4], "qy": state[5], "qz": state[6]}),
            "vel": ({"vx": state[7], "vy": state[8], "vz": state[9]}),
            "avel": ({"wx": state[10], "wy": state[11], "wz": state[12]}),
            "acc": ({"ax": state[13], "ay": state[14], "az": state[15]}),
            "aacc": ({"awx": state[16], "awy": state[17], "awz": state[18]}),
            "pos_cov": ({"cov": pos_cov}),
            "att_cov": ({"cov": att_cov}),
            "att_pos_cov": ({"cov": att_pos_cov}),
            "vel_cov": ({"cov": vel_cov}),
            "avel_cov": ({"cov": avel_cov}),
            "acc_cov": ({"cov": acc_cov}),
            "aacc_cov": ({"cov": aacc_cov})
            }
        }

    if not state_port:
        print("port 'sim_state_port' is not set")
        return
    state_port(data)

# --- rotor_speeds_from_nhfc -----------------------------------------------
def rotor_speeds_from_nhfc(c_f, n_act=4):
    desired_speeds = np.zeros(n_act)

    data = nhfc.rotor_input()["rotor_input"]
    # sometimes None may appear
    # if that happens for i-th rotor, then skip its speed
    for i, s in enumerate(data["desired"]):
        if s:
            desired_speeds[i] = data["desired"][i]
    return desired_speeds

# --- speed_to_thrust ------------------------------------------------------
def speed_to_thrust(speed: np.array, c_f):
    return np.square(speed) * c_f

# --- get_time_now_ms ------------------------------------------------------
def get_time_now_ms():
    return time.clock_gettime_ns(time.CLOCK_REALTIME)*1e-6 # return ms


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

def quaternion_to_rotation_matrix(q):
    w=q[0] 
    x=q[1] 
    y=q[2] 
    z = q[3]

    # Normalize quaternion (important!)
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

    return R

def f(x,w):
    pos=x[0:3] #position
    orient=x[3:7] #quaternon orientation
    vel=x[7:10] #linear velocity
    om=x[10:13] #angular velocity with respect to the drone
    R=quaternion_to_rotation_matrix(orient)
    G=np.array([[R[0][0],R[0][1],R[0][2],0,0,0],
                [R[1][0],R[1][1],R[1][2],0,0,0],
                [R[2][0],R[2][1],R[2][2],0,0,0],
                [0,0,0,1,0,0],
                [0,0,0,0,1,0],
                [0,0,0,0,0,1]
                ])
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


def integration(x,w,deltaT):
    k1=f(x,w)
    k2=f(x+(deltaT/2)*k1,w)
    k3=f(x+(deltaT/2)*k2,w)
    k4=f(x+(deltaT)*k3,w) 
    xnext=x+(deltaT/6)*(k1+k2*2+k3*2+k4) #rk4 integrator
    q_orient=unit_quaternon(xnext[3:7]) #making sure quaternon is unitary
    x=np.hstack((xnext[0:3],q_orient,xnext[7:10],xnext[10:13])) #getting xk+1

    return x 

################################################################################
g = genomix.connect()

g.rpath(os.environ['HOME'] + '/openrobots/lib/genom/pocolibs/plugins')

nhfc = g.load('nhfc')
maneuver = g.load('maneuver')

state_port_nhfc, state_port_maneuver = setup()

# ############################
#  INITIALIZE SIMULATION HERE
# ############################

x = np.array([0,0,0,
               1,0,0,0,
               0,0,0,
               0,0,0])
u_lambda = np.array([0,0,0,0])

t0 = 0
tf = 45
dt = 0.005

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

F = np.array([[0,0,0,0],
              [0,0,0,0],
              [cf,cf,cf,cf],
              [0,cf*L,0,-cf*L],
              [-cf*L,0,cf*L,0],
              [ct,-ct,ct,-ct]])

# preallocate arrays to store all simulation data
# more efficient than dynamic allocation
N = math.ceil((tf-t0)/dt)
tt = np.linspace(t0, tf, N)
x_log = np.zeros((N, x.shape[0]))
u_log = np.zeros((N, u_lambda.shape[0]))
t_log = np.zeros(N) # (N,)
tc_log = np.zeros(N) # (N,)

start()

# give it some time
time.sleep(0.1)

#What does WP mean here?

set_first_wp = True
set_second_wp = True
set_third_wp = True

for i, ts in enumerate(tt):
     #Instead of setting the position of the drone directly with nhfc,
     # I have used the maneuver component to set the desired trajectory for the drone and let the nhfc component follow it. 
     
    if set_first_wp and ts > 0.5:
        maneuver.set_current_state(ack=True)
        #maneuver.set_velocity_limit(v=0.3,w=0.2) #This can be used to limit the veloctities
        #maneuver.set_acceleration_limit(a=0.3,dw=0.2) #This can be used to limit the accelerations
        maneuver.goto(x=1,y=1,z=1,yaw=0, duration=10, ack=True)
        set_first_wp = False        
    elif set_second_wp and ts >= 15:
        maneuver.set_current_state(ack=True)
        maneuver.goto(x=2,y=3,z=1,yaw=0, duration=10, ack=True)
        set_second_wp = False        
    elif set_third_wp and ts >= 30:
        maneuver.set_current_state(ack=True)
        maneuver.goto(x=0,y=0,z=0,yaw=0, duration=10, ack=True)
        set_third_wp = False
    
    wrenches = F@u_lambda
    t1 = get_time_now_ms()

    x_dot=f(x,wrenches)

    x = integration(x,wrenches,dt)
    
    if x[2]<0:
        x[2]=0

    # save data
    t_log[i] = ts
    x_log[i, :] = x.reshape(-1)
    u_log[i, :] = u_lambda.reshape(-1)

    # print simulation time but every 1k iterations
    if (int(ts/dt) % 1000) == 0:
        print(f"t: {ts}")

    # if necessary, wait to match dt
    t2 = get_time_now_ms()
    elapsed_ms = t2-t1
    tc_log[i] = elapsed_ms
    if elapsed_ms > 0:
        time.sleep(elapsed_ms*1e-3)
    elif elapsed_ms < 0:
        print(f"delay of: {dt - elapsed_ms}ms")

    # update the state to both nhfc and maneuver components
    state = np.hstack((x, x_dot[-6:])).reshape(-1)
    state_to_system(state_port_nhfc,state)
    state_to_system(state_port_maneuver,state)    

    u_lambda = np.square(rotor_speeds_from_nhfc(cf))
    
    
fig,ax = plt.subplots(1,4) #plotting
ax[0].plot(t_log, x_log[:,0], color='red', label='x_pos')
ax[1].plot(t_log, x_log[:,1], color='green', label='y_pos')
ax[2].plot(t_log, x_log[:,2], color='blue',label='z_pos')    
ax[3].plot(t_log, x_log[:,5], color='yellow',label='Yaw')    

plt.show()

stop()
