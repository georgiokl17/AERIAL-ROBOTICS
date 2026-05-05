import genomix
import os
import time
import math
import numpy as np


# this connects to components running on the same host (localhost)
g = genomix.connect()
# to instead control components running on the remote computer "hostname" use
# g = genomix.connect('hostname')

# adapt path to your setup
g.rpath(os.environ['HOME'] + '/openrobots/lib/genom/pocolibs/plugins')

# load components clients
optitrack = g.load('optitrack')
rotorcraft = g.load('rotorcraft')
pom = g.load('pom')
nhfc = g.load('nhfc')
uavpos = g.load('uavpos')
uavatt = g.load('uavatt')


def get_time_now_ms():
    return time.clock_gettime_ns(time.CLOCK_REALTIME)*1e-6 # return ms
# --- setup ----------------------------------------------------------------
#
# configure components, to be called interactively
def setup():
  # optitrack
  #
  # connect to the simulated optitrack system on localhost
  optitrack.connect({
    'host': 'localhost', 'host_port': '1509', 'mcast': '', 'mcast_port': '0'
  })


  # rotorcraft
  #
  # connect to the simulated quadrotor
  rotorcraft.connect({'serial': '/tmp/pty-hr6', 'baud': 0})

  # get IMU at 1kHz and motor data at 20Hz
  rotorcraft.set_sensor_rate({'rate': {
    'imu': 1000, 'mag': 0, 'motor': 20, 'battery': 1
  }})

  # Filter IMU: 20Hz cut-off frequency for gyroscopes and 5Hz for
  # accelerometers. This is important for cancelling vibrations.
  rotorcraft.set_imu_filter({
    'gfc': [20, 20, 20], 'afc': [5, 5, 5], 'mfc': [20, 20, 20]
  })

  # read propellers velocities from nhfc controller
  rotorcraft.connect_port({
    'local': 'rotor_input', 'remote': 'uavatt/rotor_input'
  })
  

  
 

  my_reference_port = uavpos.reference('my_reference')
    # connect ports for new components
  uavpos.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot'})
  uavpos.connect_port({ 'local': 'reference', 'remote': 'my_reference'})

  uavatt.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot'})
  uavatt.connect_port({ 'local': 'uav_input', 'remote': 'uavpos/uav_input'})
  uavatt.connect_port({ 'local': 'rotor_measure', 'remote': 'rotorcraft/rotor_measure'})

  geom = {
        'rotors': 6, 'cx': 0, 'cy': 0, 'cz': 0, 'armlen': 0.40998, 'mass': 2.3,
        'rx': 2.7925, 'ry': -0.3491, 'rz': -1, 'cf': 9.9016e-4, 'ct': 1.9e-5
    }

  uavatt.set_gtmrp_geom(geom) #changed this only to have the cf as a variable we can use
  

  # pom
  #
  # configure kalman filter
  pom.set_prediction_model('::pom::constant_acceleration')
  pom.set_process_noise({'max_jerk': 100, 'max_dw': 50})

  # allow sensor data up to 250ms old
  pom.set_history_length({'history_length': 0.25})

  # configure magnetic field
  pom.set_mag_field({'magdir': {
    'x': 23.8e-06, 'y': -0.4e-06, 'z': -39.8e-06
  }})

  # read IMU and magnetometers from rotorcraft
  pom.connect_port({'local': 'measure/imu', 'remote': 'rotorcraft/imu'})
  pom.add_measurement('imu')
  pom.connect_port({'local': 'measure/mag', 'remote': 'rotorcraft/mag'})
  pom.add_measurement('mag')

  # read position and orientation from optitrack
  pom.connect_port({
    'local': 'measure/mocap', 'remote': 'optitrack/bodies/HR_6'
  })
  pom.add_measurement('mocap')


  variance_imu=rotorcraft.get_imu_calibration()
  print(variance_imu)

  

  rotorcraft.set_imu_calibration(variance_imu)

  return my_reference_port

reference_port_uavpos = setup()

#Modified this from state_to_nhfc to be able to send the state of the drone to both nhfc and maneuver components

def reference_to_uavpos(reference_port, state: np.array):
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
            "jerk": {"jx": state[19], "jy": state[20], "jz": state[21]},
            "snap": {"sx": state[22], "sy": state[23], "sz": state[24]},
            }
        }

    if not reference_port:
        print("port 'sim_state_port' is not set")
        return
    reference_port(data)


def quaternion_to_euler(q):
    """
    Converts a quaternion [qx, qy, qz, qw] into roll, pitch, and yaw (in radians).
    """
    qx, qy, qz, qw = q

    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

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


def rpy_to_quaternion(roll, pitch, yaw):
    cr = np.cos(roll / 2)
    sr = np.sin(roll / 2)
    cp = np.cos(pitch / 2)
    sp = np.sin(pitch / 2)
    cy = np.cos(yaw / 2)
    sy = np.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])


def traj_plan(x_current,x_desired,duration,dt):

    x_now=x_current[0]
    y_now=x_current[1]
    z_now=x_current[2]
    roll_now=x_current[3]
    pitch_now=x_current[4]
    yaw_now=x_current[5]


    x_final=x_desired[0]
    y_final=x_desired[1]
    z_final=x_desired[2]
    roll_final=x_desired[3]
    pitch_final=x_desired[4]
    yaw_final=x_desired[5]

    iteration = int(duration / dt)
    
    vx_now=0
    vy_now=0
    vz_now=0
    vroll_now=0
    vpitch_now=0
    vyaw_now=0

    ax_now=0
    ay_now=0
    az_now=0
    aroll_now=0
    apitch_now=0
    ayaw_now=0


    vx_final=0
    vy_final=0
    vz_final=0
    vroll_final=0
    vpitch_final=0
    vyaw_final=0

    ax_final=0
    ay_final=0
    az_final=0
    aroll_final=0
    apitch_final=0
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
    Broll=np.array([roll_now,vroll_now,aroll_now,roll_final,vroll_final, aroll_final])
    Bpitch=np.array([pitch_now,vpitch_now,apitch_now,pitch_final,vpitch_final, apitch_final])
    Byaw=np.array([yaw_now,vyaw_now,ayaw_now,yaw_final,vyaw_final, ayaw_final])

    coeff_x = np.linalg.solve(A, Bx)
    coeff_y = np.linalg.solve(A, By)
    coeff_z = np.linalg.solve(A, Bz)
    coeff_roll = np.linalg.solve(A, Broll)
    coeff_pitch = np.linalg.solve(A, Bpitch)
    coeff_yaw = np.linalg.solve(A, Byaw)

    q_now=rpy_to_quaternion(roll_now,pitch_now,yaw_now)
    qw_now=q_now[0]
    qx_now=q_now[1]
    qy_now=q_now[2]
    qz_now=q_now[3]
    state_now = np.array([x_now,y_now,z_now,qw_now,qx_now,qy_now,qz_now,
                          vx_now,vy_now,vz_now,vroll_now,vpitch_now,vyaw_now,
                          ax_now,ay_now,az_now,0,0,0,
                          0,0,0,0,0,0])

    state_pos = { 
            "pos": ({"x": state_now[0], "y": state_now[1], "z": state_now[2]}),
            "att": ({"qw": state_now[3], "qx": state_now[4], "qy": state_now[5], "qz": state_now[6]}),
            "vel": ({"vx": state_now[7], "vy": state_now[8], "vz": state_now[9]}),
            "avel": ({"wx": state_now[10], "wy": state_now[11], "wz": state_now[12]}),
            "acc": ({"ax": state_now[13], "ay": state_now[14], "az": state_now[15]}),
            "aacc": ({"awx": state_now[16], "awy": state_now[17], "awz": state_now[18]}),
            "jerk": {"jx": state_now[19], "jy": state_now[20], "jz": state_now[21]},
            "snap": {"sx": state_now[22], "sy": state_now[23], "sz": state_now[24]},
            }
        
    state_att = { 
            "thrust": ({"x": state_now[0], "y": state_now[1], "z": state_now[2]}),
            "att": ({"qw": state_now[3], "qx": state_now[4], "qy": state_now[5], "qz": state_now[6]}),
            "avel": ({"wx": state_now[10], "wy": state_now[11], "wz": state_now[12]}),
            "aacc": ({"awx": state_now[16], "awy": state_now[17], "awz": state_now[18]}),
            }
    uavpos.set_state(state_pos)
    uavatt.set_state(state_att)

    for i in range(iteration):
        t1 = get_time_now_ms()
        nt=i*dt
        
        
        xt=coeff_x[0]+coeff_x[1]*nt+coeff_x[2]*nt**2+coeff_x[3]*nt**3+coeff_x[4]*nt**4 +coeff_x[5]*nt**5
        yt=coeff_y[0]+coeff_y[1]*nt+coeff_y[2]*nt**2+coeff_y[3]*nt**3+coeff_y[4]*nt**4 +coeff_y[5]*nt**5
        zt=coeff_z[0]+coeff_z[1]*nt+coeff_z[2]*nt**2+coeff_z[3]*nt**3+coeff_z[4]*nt**4 +coeff_z[5]*nt**5
        rollt=coeff_roll[0]+coeff_roll[1]*nt+coeff_roll[2]*nt**2+coeff_roll[3]*nt**3+coeff_roll[4]*nt**4 +coeff_roll[5]*nt**5
        pitcht=coeff_pitch[0]+coeff_pitch[1]*nt+coeff_pitch[2]*nt**2+coeff_pitch[3]*nt**3+coeff_pitch[4]*nt**4 +coeff_pitch[5]*nt**5
        yawt=coeff_yaw[0]+coeff_yaw[1]*nt+coeff_yaw[2]*nt**2+coeff_yaw[3]*nt**3+coeff_yaw[4]*nt**4 +coeff_yaw[5]*nt**5

        vxt=coeff_x[1]+2*coeff_x[2]*nt+3*coeff_x[3]*nt**2+4*coeff_x[4]*nt**3 +5*coeff_x[5]*nt**4
        vyt=coeff_y[1]+2*coeff_y[2]*nt+3*coeff_y[3]*nt**2+4*coeff_y[4]*nt**3 +5*coeff_y[5]*nt**4
        vzt=coeff_z[1]+2*coeff_z[2]*nt+3*coeff_z[3]*nt**2+4*coeff_z[4]*nt**3 +5*coeff_z[5]*nt**4
        vrollt=coeff_roll[1]+2*coeff_roll[2]*nt+3*coeff_roll[3]*nt**2+4*coeff_roll[4]*nt**3 +5*coeff_roll[5]*nt**4
        vpitcht=coeff_pitch[1]+2*coeff_pitch[2]*nt+3*coeff_pitch[3]*nt**2+4*coeff_pitch[4]*nt**3 +5*coeff_pitch[5]*nt**4
        vyawt=coeff_yaw[1]+2*coeff_yaw[2]*nt+3*coeff_yaw[3]*nt**2+4*coeff_yaw[4]*nt**3 +5*coeff_yaw[5]*nt**4

        axt=2*coeff_x[2]+2*3*coeff_x[3]*nt+3*4*coeff_x[4]*nt**2 +4*5*coeff_x[5]*nt**3
        ayt=2*coeff_y[2]+2*3*coeff_y[3]*nt+3*4*coeff_y[4]*nt**2 +4*5*coeff_y[5]*nt**3
        azt=2*coeff_z[2]+2*3*coeff_z[3]*nt+3*4*coeff_z[4]*nt**2 +4*5*coeff_z[5]*nt**3
        arollt=2*coeff_roll[2]+2*3*coeff_roll[3]*nt+3*4*coeff_roll[4]*nt**2 +4*5*coeff_roll[5]*nt**3
        apitcht=2*coeff_pitch[2]+2*3*coeff_pitch[3]*nt+3*4*coeff_pitch[4]*nt**2 +4*5*coeff_pitch[5]*nt**3
        ayawt=2*coeff_yaw[2]+2*3*coeff_yaw[3]*nt+3*4*coeff_yaw[4]*nt**2 +4*5*coeff_yaw[5]*nt**3

        jerk_x=2*3*coeff_x[3]+2*3*4*coeff_x[4]*nt +3*4*5*coeff_x[5]*nt**2
        jerk_y=2*3*coeff_y[3]+2*3*4*coeff_y[4]*nt +3*4*5*coeff_y[5]*nt**2
        jerk_z=2*3*coeff_z[3]+2*3*4*coeff_z[4]*nt +3*4*5*coeff_z[5]*nt**2

        snap_x=2*3*4*coeff_x[4] +2*3*4*5*coeff_x[5]*nt
        snap_y=2*3*4*coeff_y[4] +2*3*4*5*coeff_y[5]*nt
        snap_z=2*3*4*coeff_z[4] +2*3*4*5*coeff_z[5]*nt


        q=rpy_to_quaternion(rollt,pitcht,yawt)
        qw=q[0]
        qx=q[1]
        qy=q[2]
        qz=q[3]

        xh=np.array([xt,yt,zt,  #state here in the function 
           qw,qx,qy,qz,
           vxt,vyt,vzt,
           vrollt,vpitcht,vyawt])
        orient=np.array([qw,qx,qy,qz])
        R=quaternion_to_rotation_matrix(orient)
        om=np.array([vrollt,vpitcht,vyawt])
        om_w=R@om
        qomg_W=np.hstack((0, om_w))
        orient_dot=0.5*quaternon_mult(qomg_W,orient)
        xdot=np.array([vxt,vyt,vzt,                             #xdot defined based on results of function 
                       orient_dot[0],orient_dot[1],orient_dot[2],orient_dot[3],
                       axt,ayt,azt,
                       arollt,apitcht,ayawt])
        jerk_snap=np.array([jerk_x,jerk_y,jerk_z,snap_x,snap_y,snap_z])
        state = np.hstack((xh, xdot[-6:],jerk_snap))

        t2 = get_time_now_ms()
        elapsed_time = t2-t1
        time.sleep(dt-elapsed_time*1e-3)
        print(f"delay of: {elapsed_time}ms")
        reference_to_uavpos(reference_port_uavpos,state)

    

        ct=nt

# Function that make the quadrotor follow the given trajectory
def move():
    state = pom.frame('robot')['frame']

    pos = state['pos']
    att = state['att']

    current_state_pos = np.array([pos['x'], pos['y'], pos['z']])
    current_state_att = np.array([att['qx'], att['qy'], att['qz'], att['qw']])

    euler=quaternion_to_euler(current_state_att)
    


    # current_state = 
    xnow=np.hstack((current_state_pos,euler))
    xfinal=[1,1,1,0,0,0]
    duration=5
    dt=0.01
    time.sleep(2)
    traj_plan(xnow,xfinal,duration,dt)
    #control command 1
    time.sleep(10)
    traj_plan(xfinal,xnow,duration,dt)
    #control command 2

# --- start ----------------------------------------------------------------
#
# Spin the motors and servo on current position. To be called interactively
# I have changed the folder to file path to save the log files
# The below has been changed to save

def start():
  pom.log_state('../logs/01_Quadrotor/pom.log')
  pom.log_measurements('../logs/01_Quadrotor/pom-measurements.log')

  optitrack.set_logfile('../logs/01_Quadrotor/opti.log')

  rotorcraft.log('../logs/01_Quadrotor/rotorcraft.log')
  rotorcraft.start()
  time.sleep(2)

  uavpos.servo(ack=True)
  uavatt.servo(ack=True)

  rotorcraft.servo(ack=True) # this runs until stopped or input error

  
  move()



# --- stop -----------------------------------------------------------------
#
# Stop motors. To be called interactively
def stop():
  rotorcraft.stop()
  rotorcraft.log_stop()


  pom.log_stop()

  optitrack.unset_logfile()


## interactively, one can start the simulation with
# setup()
# start()
## and then for instance set a desired position with
# nhfc.set_position(0, 0, 1, 0)
## to stop, use
# stop


def simulation():

  start()
  stop()
  
    
