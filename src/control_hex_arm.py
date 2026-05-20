import genomix
import os
import time
import math
import numpy as np
import matplotlib.pyplot as plt

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
maneuver = g.load('maneuver')
dynamixel = g.load('dynamixel')
phynt = g.load('phynt')

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

  dynamixel.connect({  #connecting the dynamix model
    'serial': '/tmp/pty-dynamixel',  #since that is the serial we created in the plugin of hexa arm
    'baud': 1000000  #baud rate
  })
  dynamixel.set_position({  #setting fixed initial position of motor
    'position': [0]
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
  
  # UAVPOS SETTINGS:
   # --- uavpos ---
  uavpos.set_mass({'mass': 2.55})

  uavpos.set_xyradius({'rxy': 2.0})   # from the slide guideline

  uavpos.set_saturation({'sat': {
        'x': 1,
        'v': 1,
        'ix': 0
  }})
  
  uavpos.set_servo_gain({'gain': {
        'Kpxy': 20,
        'Kpz': 50,
        'Kvxy': 6.0,
        'Kvz': 6.0,
        'Kixy': 20,
        'Kiz': 80.0
  }})

  uavpos.set_emerg({'emerg': {
        'descent': 0.1,
        'dx': 0.05,
        'dv': 0.2
  }})
  # UAVATT SETTINGS:
  uavatt.set_mass({'mass': 2.55})

  uavatt.set_wlimit({
        'wmin': 0.0,
        'wmax': 120.0
  })


  uavatt.set_servo_gain({'gain': {
        'Kqxy': 60,
        'Kqz': 60,
        'Kwxy': 4,
        'Kwz': 4
  }})

  uavatt.set_emerg({'emerg': {
        'dq': 20,
        'dw': 20
  }})

 

   # connect ports for new components
  maneuver.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot' })
  phynt.connect_port({'local': 'state', 'remote': 'pom/frame/robot'})
  phynt.connect_port({'local': 'reference', 'remote': 'maneuver/desired'})

  phynt.connect_port({'local': 'wrench_measure', 'remote': 'uavatt/wrench_measure'})  
  
  uavpos.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot'})

  uavpos.connect_port({'local': 'reference', 'remote': 'phynt/desired'})
  uavatt.connect_port({ 'local': 'state', 'remote': 'pom/frame/robot'})
  uavatt.connect_port({ 'local': 'uav_input', 'remote': 'uavpos/uav_input'})
  uavatt.connect_port({ 'local': 'rotor_measure', 'remote': 'rotorcraft/rotor_measure'})

  geom = {
        'rotors': 6, 'cx': 0, 'cy': 0.0, 'cz': 0.0, 'armlen': 0.40998, 'mass': 2.55,
        'rx': -20, 'ry': 0, 'rz': -1, 'cf': 9.9016e-4, 'ct': 1.9e-5
    }
  
  Jxx = 0.0136+0.1*((0.15*math.cos(0))**2+(-0.1-0.15*math.sin(0))**2)
  Jyy = 0.0124 + 0.1*(-0.1-0.15*math.sin(0))**2
  Jzz = 0.0204 +0.1*(0.15*math.cos(0))**2
 
  phynt.set_mass({'mass': 2.55})
  phynt.set_geom({'J': [
        Jxx, 0,     0,
        0,     Jyy, 0,
        0,     0,     Jzz] })
  
  phynt.set_af_parameters({
        'mass': 4.0,   # apparent mass, larger than real mass
        'B': [2*50*4, 2*50*4, 2*50*4, 
              2*5*4, 2*5*4, 2*5*4],
        'K': [50, 10, 50, 
              5, 5, 5],
        'J': [Jxx+0.005, 0,     0,
              0,    Jyy+0.005, 0,
              0,     0,   Jzz+0.005]
        
    })
  phynt.set_wo_gains({ #idk all these from chat gpt and documentation
        'K': [1, 1, 1, 1, 1, 1]
    })

  phynt.set_wo_fc({
      'fc': [20, 20, 20, 20, 20, 20]
  })

  phynt.enable({
      'enable': {
          'wo': True,
          'af': True
      }})
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
  #print(variance_imu)

  

  rotorcraft.set_imu_calibration(variance_imu)

  
#i wann 
#Modified this from state_to_nhfc to be able to send the state of the drone to both nhfc and maneuver components




state = pom.frame('robot')['frame']
pos = state['pos']

print(pos['x'])
print(pos['z'])
# Function that make the quadrotor follow the given trajectory
def move():
    angle_motor=-0.5
    offset_y = math.cos(angle_motor)*0.3
    offset_z = math.sin(angle_motor)*0.3-0.1-0.025-0.02
    x_contact=1.5
    y_contact=0
    z_contact=0
    x_drone = x_contact + offset_y
    y_drone = y_contact 
    z_drone = z_contact - offset_z 
    print('it should go to this z position:',z_drone)
    print('it should go to this y position:',y_drone)
    print('it should go to this x position:',x_drone)
    
    maneuver.set_bounds(xmin=-5,xmax=5,ymin=-5,ymax=5,zmin=0,zmax=5,yawmin=0,yawmax=3.14) #setting bounds for the maneuver component to make sure the drone does not go out of a certain area)

    maneuver.set_current_state() 

    time.sleep(2)
    
    maneuver.goto(x=2.5,y=0,z=2,yaw=3.14/2, duration=10, send=True, ack=True)
    #control command 2
    desired_phynt = phynt.desired()['desired']['pos']
    print('this is the first desired pos', desired_phynt) 

    time.sleep(20)
    
    dynamixel.set_position({  #setting fixed initial position of motor
    'position': [angle_motor]
      })

    maneuver.goto(x=x_drone,y=y_drone,z=z_drone,yaw=3.14/2, duration=10, send=True, ack=True)
  

    time.sleep(10)
    desired_phynt2 = phynt.desired()['desired']['pos']
    print('this is the second desired pos', desired_phynt2) 
    

    state2 = pom.frame('robot')['frame']
    pos2 = state2['pos']
    print('It is going to this z',pos2['z'])
    print('It is going to this y',pos2['y'])
    print('It is going to this x',pos2['x'])
    maneuver.goto(x=-1,y=y_drone,z=z_drone,yaw=3.14/2, duration=10, send=True, ack=True)

    time.sleep(10)

# --- start ----------------------------------------------------------------
#
# Spin the motors and servo on current position. To be called interactively
# I have changed the folder to file path to save the log files
# The below has been changed to save

def start():
  #pom.log_state('../logs/01_Quadrotor/pom.log')
  #pom.log_measurements('../logs/01_Quadrotor/pom-measurements.log')

  #set_logfile('../logs/01_Quadrotor/opti.log')

  #rotorcraft.log('../logs/01_Quadrotor/rotorcraft.log')
  rotorcraft.start()
  phynt.log('../phynt.log')
  time.sleep(0.5)

  uavpos.set_current_position() # must be before servo otherwise it interrupts the servo activity

  phynt.servo(ack=True)
  uavpos.servo(ack=True)
  uavatt.servo(ack=True)
  rotorcraft.servo(ack=True) # this runs until stopped or input error

  
  move()



# --- stop -----------------------------------------------------------------
#
# Stop motors. To be called interactively
def stop():
  rotorcraft.stop()
  phynt.log_stop()


  pom.log_stop()

  optitrack.unset_logfile()
  phynt.stop()
  uavatt.stop()
  maneuver.stop()
  uavpos.stop()


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