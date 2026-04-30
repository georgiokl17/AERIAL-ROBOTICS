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
    'local': 'rotor_input', 'remote': 'nhfc/rotor_input'
  })
  #Created a port my_state_man where maneuver component will receive the state of the drone from our simulator
  my_state_port_maneuver = maneuver.state('my_state_man')
   
  # connect maneuver to state port which will be updated by our simulator
  maneuver.connect_port({ 'local': 'state', 'remote': 'my_state_man' })


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



# Function that make the quadrotor follow the given trajectory
def move():
    time.sleep(2) 
    # Removed the 'pos' wrapper - passing the coordinates directly
    nhfc.set_position({'x': 0, 'y': 0, 'z': 1, 'yaw': 0}) 
    
    time.sleep(5)
    nhfc.set_position({'x': 0, 'y': 0, 'z': 0, 'yaw': 0})
    time.sleep(3)
    

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
  
    
