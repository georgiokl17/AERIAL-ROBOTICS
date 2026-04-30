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
  
    
    my_state_port_nhfc = nhfc.state('my_state_nhfc')

    my_reference_port = nhfc.reference('my_reference') #creating port for reference trajectory
    
    
    # connect nhfc to state port which will be updated by our simulator
    nhfc.connect_port({ 'local': 'state', 'remote': 'my_state_nhfc' })
    nhfc.connect_port({'local': 'reference', 'remote': 'my_reference'}) #connecting reference port to nhfc
    
    return  my_state_port_nhfc, my_reference_port

def taj_plan(x_current,x_desired,duration):

    x_now=x_current[0]
    y_now=x_current[1]
    z_now=x_current[2]
    yaw_now=x_current[3]

    x_final=x_desired[0]
    y_final=x_desired[1]
    z_final=x_desired[2]
    yaw_final=x_desired[3]

    iteration = int(duration / dt)
    
    vx_now=0
    vy_now=0
    vz_now=0
    vyaw_now=0

    ax_now=0
    ay_now=0
    az_now=0
    ayaw_now=0


    vx_final=0
    vy_final=0
    vz_final=0
    vyaw_final=0

    ax_final=0
    ay_final=0
    az_final=0
    ayaw_final=0


    ct=0
    nt=duration


    A=np.array([[1, ct, ct**2, ct**3, ct**4, ct**5],       #for x y z yaw initial points
                [0, 1, 2*ct, 3*ct**2, 4*ct**3, 5*ct**4],   #for xdot ydot ... initial points
                [0, 0, 2, 2*3*ct, 3*4*ct**2, 4*5*ct**3],   #for xdouble dot ... initial points
                [1, nt, nt**2, nt**3, nt**4, nt**5],        #for x y z yaw final points
                [0, 1, 2*nt, 3*nt**2, 4*nt**3, 5*nt**4],    #for xdot ydot ... ifinal points
                [0, 0, 2, 2*3*nt, 3*4*nt**2, 4*5*nt**3]])   #for xdouble dot ... final points
        
    Bx=np.array([x_now,vx_now,ax_now,x_final,vx_final, ax_final])
    By=np.array([y_now,vy_now,ay_now,y_final,vy_final, ay_final])
    Bz=np.array([z_now,vz_now,az_now,z_final,vz_final, az_final])
    Byaw=np.array([yaw_now,vyaw_now,ayaw_now,yaw_final,vyaw_final, ayaw_final])

    coeff_x = np.linalg.solve(A, Bx)
    coeff_y = np.linalg.solve(A, By)
    coeff_z = np.linalg.solve(A, Bz)
    coeff_yaw = np.linalg.solve(A, Byaw)

    state_ls=[]
    

    for i in range(iteration):

        nt=i*dt
        
        
        xt=coeff_x[0]+coeff_x[1]*nt+coeff_x[2]*nt**2+coeff_x[3]*nt**3+coeff_x[4]*nt**4 +coeff_x[5]*nt**5
        yt=coeff_y[0]+coeff_y[1]*nt+coeff_y[2]*nt**2+coeff_y[3]*nt**3+coeff_y[4]*nt**4 +coeff_y[5]*nt**5
        zt=coeff_z[0]+coeff_z[1]*nt+coeff_z[2]*nt**2+coeff_z[3]*nt**3+coeff_z[4]*nt**4 +coeff_z[5]*nt**5
        yawt=coeff_yaw[0]+coeff_yaw[1]*nt+coeff_yaw[2]*nt**2+coeff_yaw[3]*nt**3+coeff_yaw[4]*nt**4 +coeff_yaw[5]*nt**5

        vxt=coeff_x[1]+2*coeff_x[2]*nt+3*coeff_x[3]*nt**2+4*coeff_x[4]*nt**3 +5*coeff_x[5]*nt**4
        vyt=coeff_y[1]+2*coeff_y[2]*nt+3*coeff_y[3]*nt**2+4*coeff_y[4]*nt**3 +5*coeff_y[5]*nt**4
        vzt=coeff_z[1]+2*coeff_z[2]*nt+3*coeff_z[3]*nt**2+4*coeff_z[4]*nt**3 +5*coeff_z[5]*nt**4
        vyawt=coeff_yaw[1]+2*coeff_yaw[2]*nt+3*coeff_yaw[3]*nt**2+4*coeff_yaw[4]*nt**3 +5*coeff_yaw[5]*nt**4

        axt=2*coeff_x[2]+2*3*coeff_x[3]*nt+3*4*coeff_x[4]*nt**2 +4*5*coeff_x[5]*nt**3
        ayt=2*coeff_y[2]+2*3*coeff_y[3]*nt+3*4*coeff_y[4]*nt**2 +4*5*coeff_y[5]*nt**3
        azt=2*coeff_z[2]+2*3*coeff_z[3]*nt+3*4*coeff_z[4]*nt**2 +4*5*coeff_z[5]*nt**3
        ayawt=2*coeff_yaw[2]+2*3*coeff_yaw[3]*nt+3*4*coeff_yaw[4]*nt**2 +4*5*coeff_yaw[5]*nt**3

        qw = math.cos(yawt / 2)
        qx = 0
        qy = 0
        qz = math.sin(yawt / 2)

        xh=np.array([xt,yt,zt,  #state here in the function 
           qw,qx,qy,qz,
           vxt,vyt,vzt,
           0,0,vyawt])
        orient=np.array([qw,qx,qy,qz])
        R=quaternion_to_rotation_matrix(orient)
        om=np.array([0,0,yawt])
        om_w=R@om
        qomg_W=np.hstack((0, om_w))
        orient_dot=0.5*quaternon_mult(qomg_W,orient)
        xdot=np.array([vxt,vyt,vzt,                             #xdot defined based on results of function 
                       orient_dot[0],orient_dot[1],orient_dot[2],orient_dot[3],
                       axt,ayt,azt,
                       0,0,ayawt])
        state = np.hstack((xh, xdot[-6:])).reshape(-1)
        state_ls.append(state)

    

        ct=nt

    return state_ls

        






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


def reference_to_nhfc(reference_port, state: np.array):
    def _get_time():
        # returns a tuple of the type (<sec>,<nsec>)
        now = math.modf(time.clock_gettime(time.CLOCK_REALTIME))
        return (int(now[1]), int(now[0]*1e9))


    now = _get_time()

    # port and message descriptions at
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/pose_estimator.gen
    # https://git.openrobots.org/projects/openrobots-idl/repository/openrobots-idl/revisions/master/entry/pose/t3d.idl
    data = { "reference": {
            "ts" : {"sec": now[0], "nsec": now[1]},
            "intrinsic": False,
            "pos": ({"x": state[0], "y": state[1], "z": state[2]}),
            "att": ({"qw": state[3], "qx": state[4], "qy": state[5], "qz": state[6]}),
            "vel": ({"vx": state[7], "vy": state[8], "vz": state[9]}),
            "avel": ({"wx": state[10], "wy": state[11], "wz": state[12]}),
            "acc": ({"ax": state[13], "ay": state[14], "az": state[15]}),
            "aacc": ({"awx": state[16], "awy": state[17], "awz": state[18]}),
            "jerk": {"jx": 0, "jy": 0, "jz": 0},
            "snap": {"sx": 0, "sy": 0, "sz": 0},
            }
        }

    if not reference_port:
        print("port 'sim_state_port' is not set")
        return
    reference_port(data)

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

    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ])

    return R

def get_yaw(w, x, y, z):
    # Formula for Yaw (rotation around Z-axis)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

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
    
    om_W=R@om #angular velocity with respect to the world
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

state_port_nhfc, reference_port_nhfc = setup()

# ############################
#  INITIALIZE SIMULATION HERE
# ############################

x = np.array([0,0,0,
               1,0,0,0,
               0,0,0,
               0,0,0])
u_lambda = np.array([0,0,0,0])

t0 = 0
tf = 30
dt = 0.1

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
L=0.23 #distance of rotor from center of mass of drone

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
yaw_log = np.zeros(N)
u_log = np.zeros((N, u_lambda.shape[0]))
t_log = np.zeros(N) # (N,)
tc_log = np.zeros(N) # (N,)

start()

# give it some time
time.sleep(0.1)

x_curr=[0,0,0,0]
x_des=[1,1,1,0]
# set_fourth_wp = True
des_ls=taj_plan(x_curr,x_des,tf)  #desired points list

for i, ts in enumerate(tt):
     #Instead of setting the position of the drone directly with nhfc,
     # I have used the maneuver component to set the desired trajectory for the drone and let the nhfc component follow it. 
    
    
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
    

    # update the state to both nhfc and maneuver components
    state = np.hstack((x, x_dot[-6:])).reshape(-1)
    state2 = des_ls[i]
    state_to_system(state_port_nhfc,state)
    reference_to_nhfc(reference_port_nhfc ,state2)  
    yaw_val = get_yaw(x[3],x[4],x[5],x[6])
    yaw_log[i] = yaw_val  

    u_lambda = np.square(rotor_speeds_from_nhfc(cf))
    
    t2 = get_time_now_ms()
    elapsed_ms = t2-t1
    tc_log[i] = elapsed_ms
    sleep_time= dt - elapsed_ms*1e-3
    # if elapsed_ms > 0:
    time.sleep(sleep_time)
    # elif elapsed_ms < 0:
    print(f"delay of: {elapsed_ms}ms")
    
fig,ax = plt.subplots(1,4) #plotting
ax[0].plot(t_log, x_log[:,0], color='red', label='x_pos')
ax[1].plot(t_log, x_log[:,1], color='green', label='y_pos')
ax[2].plot(t_log, x_log[:,2], color='blue',label='z_pos')    
ax[3].plot(t_log, yaw_log, color='yellow',label='Yaw')      
  
ax[0].grid(True)
ax[1].grid(True)
ax[2].grid(True)
ax[3].grid(True)
print(f"the max y val is:{np.max(x_log[:,1])}")
plt.show()

stop()