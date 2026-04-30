import genomix
import os
import time
#this is my comment ~georgio
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
  #Created a port my_state_man where maneuver component will receive the state of the drone from our simulator
  #my_state_port_maneuver = maneuver.state('my_state_man')
   
  # connect maneuver to state port which will be updated by our simulator
  maneuver.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot'})
  
 

 my_reference_port = uavpos.reference('my_reference')
  # connect ports for new components
  uavpos.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot'})
  uavpos.connect_port({ 'local': 'reference', 'remote': 'manuever/desired'})

  uavatt.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot'})
  uavatt.connect_port({ 'local': 'uav_input', 'remote': 'uavpos/uav_input'})
  uavatt.connect_port({ 'local': 'rotor_measure', 'remote': 'rotorcraft/rotor_measure'})

#   # nhfc
#   #
#   # configure quadrotor geometry: 4 rotors, not tilted, 23cm arms
#   geom = {
#     'rotors': 4, 'cx': 0, 'cy': 0, 'cz': 0, 'armlen': 0.23, 'mass': 1.28,
#     'rx': 0, 'ry': 0, 'rz': -1, 'cf': 6.5e-4, 'ct': 1e-5
#   }

#   nhfc.set_gtmrp_geom(geom) #changed this only to have the cf as a variable we can use
  

#   # emergency descent parameters
#   nhfc.set_emerg({'emerg': {
#     'descent': 0.1, 'dx': 0.5, 'dq': 1, 'dv': 3, 'dw': 3
#   }})
  
#   # PID tuning
#   nhfc.set_saturation({'sat': {'x': 1, 'v': 1, 'ix': 0}})
#   nhfc.set_servo_gain({ 'gain': {
#     'Kpxy': 5, 'Kpz': 5, 'Kqxy': 4, 'Kqz': 0.1,
#     'Kvxy': 6, 'Kvz': 6, 'Kwxy': 1, 'Kwz': 0.1,
#     'Kixy': 0, 'Kiz': 0
#   }})

#   # use tilt-prioritized controller
#   nhfc.set_control_mode({'att_mode': '::nhfc::tilt_prioritized'})

#   # read measured propeller velocities from rotorcraft
#   nhfc.connect_port({
#     'local': 'rotor_measure', 'remote': 'rotorcraft/rotor_measure'
#   })

#   # read current state from pom
#   nhfc.connect_port({
#     'local': 'state', 'remote': 'pom/frame/robot'
#   })


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
    'local': 'measure/mocap', 'remote': 'optitrack/bodies/QR_4'
  })
  pom.add_measurement('mocap')


  variance_imu=rotorcraft.get_imu_calibration()
  print(variance_imu)

  # variance_imu['imu_calibration']['astddev'] = [0.05,0.05,0.05]  #accelerometer noise
  # variance_imu['imu_calibration']['gstddev'] = [0.01,0.01,0.01]  #gyroscope noise
  # variance_imu['imu_calibration']['mstddev'] = [0.05,0.05,0.05]  #magnetometer noise

  rotorcraft.set_imu_calibration(variance_imu)


  #optitrack.set_noise(0.05,0.05)


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



# Function that make the quadrotor follow the given trajectory
def move():
    current_state_pos = np.zeros(3)
    current_state_pos = pom.measure()['pos']
    current_state_att = np.zeros(4)
    current_state_att = pom.measure()['att']


    current_state = 
    
    time.sleep(2)
    maneuver.set_state(x=0,y=0,z=0,yaw=0) 
    maneuver.take_off(height=1, duration=5, send=True, ack=True)
    #control command 1
    time.sleep(10)
    maneuver.set_current_state()
    maneuver.goto(x=5,y=5,z=5,yaw=0.2, duration=10, send=True, ack=True)
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
  rotorcraft.servo(ack=True) # this runs until stopped or input error

  nhfc.log('../logs/01_Quadrotor/nhfc.log') #added nhfc log to get errors
  nhfc.set_current_position() # hover on current position
  move()



# --- stop -----------------------------------------------------------------
#
# Stop motors. To be called interactively
def stop():
  rotorcraft.stop()
  rotorcraft.log_stop()

  nhfc.stop()
  nhfc.log_stop()

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
  setup()
  start()
  stop()
  
    
