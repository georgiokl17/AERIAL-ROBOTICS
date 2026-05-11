import genomix
import os
import time
import math

g = genomix.connect()
g.rpath(os.environ['HOME'] + '/openrobots/lib/genom/pocolibs/plugins')

optitrack = g.load('optitrack')
rotorcraft = g.load('rotorcraft')
pom = g.load('pom')
nhfc = g.load('nhfc')

LOG_DIR = os.path.join(os.environ['TK3LAB_WS'], 'logs', 'fahexa_nhfc')
os.makedirs(LOG_DIR, exist_ok=True)


def setup():
    # --- optitrack ---
    optitrack.connect({
        'host': 'localhost',
        'host_port': '1509',
        'mcast': '',
        'mcast_port': '0'
    })

    # --- rotorcraft ---
    # Cambia questo path se il tuo world crea un nome diverso
    rotorcraft.connect({'serial': '/tmp/pty-fahexa4', 'baud': 0})

    rotorcraft.set_sensor_rate({'rate': {
        'imu': 1000,
        'mag': 0,
        'motor': 20,
        'battery': 1
    }})

    rotorcraft.set_imu_filter({
        'gfc': [20, 20, 20],
        'afc': [5, 5, 5],
        'mfc': [20, 20, 20]
    })

    rotorcraft.connect_port({
        'local': 'rotor_input',
        'remote': 'nhfc/rotor_input'
    })

    # --- nhfc ---
    # GTMRP = tilted multi-rotor platform
    # rx/ry are in DEGREES according to the doc you sent
    nhfc.set_gtmrp_geom({
        'rotors': 6,
        'cx': 0,
        'cy': 0,
        'cz': 0,
        'armlen': 0.38998,
        'mass': 2.72,
        'mbodyw': 0.1354,
        'mbodyh': 0.05,
        'mmotor': 0.07,
        'rx': 20,      # rotor tilt angle in degrees
        'ry': 0,
        'rz': -1,      # first rotor spin direction: -1 = cw
        'cf': 2e-3,
        'ct': 1.9e-5
    })

    nhfc.set_mass({'mass': 2.72})

    nhfc.set_wlimit({
        'wmin': 0.0,
        'wmax': 120.0
    })

    nhfc.set_saturation({'sat': {
        'x': 1,
        'v': 1,
        'ix': 0
    }})



    nhfc.set_servo_gain({'gain': {
        'Kpxy': 5,
        'Kpz': 5,
        'Kqxy': 4,
        'Kqz': 0.1,
        'Kvxy': 6,
        'Kvz': 6,
        'Kwxy': 1,
        'Kwz': 0.1,
        'Kixy': 0,
        'Kiz': 0
    }})

    nhfc.set_control_mode({'att_mode': '::nhfc::tilt_prioritized'})

    nhfc.set_emerg({'emerg': {
        'descent': 0.1,
        'dx': 0.05,
        'dq': 20,
        'dv': 0.2,
        'dw': 20
    }})

    nhfc.connect_port({
        'local': 'rotor_measure',
        'remote': 'rotorcraft/rotor_measure'
    })

    nhfc.connect_port({
        'local': 'state',
        'remote': 'pom/frame/robot'
    })

    # --- pom ---
    pom.set_prediction_model('::pom::constant_acceleration')
    pom.set_process_noise({'max_jerk': 100, 'max_dw': 50})
    pom.set_history_length({'history_length': 0.25})

    pom.set_mag_field({'magdir': {
        'x': 23.8e-06,
        'y': -0.4e-06,
        'z': -39.8e-06
    }})

    pom.connect_port({'local': 'measure/imu', 'remote': 'rotorcraft/imu'})
    pom.add_measurement('imu')

    pom.connect_port({'local': 'measure/mag', 'remote': 'rotorcraft/mag'})
    pom.add_measurement('mag')

    # Cambia HR_6 se nel world hai pubblicato un nome diverso
    pom.connect_port({
        'local': 'measure/mocap',
        'remote': 'optitrack/bodies/FH_4'
    })
    pom.add_measurement('mocap')


def start():
    pom.log_state(os.path.join(LOG_DIR, 'pom.log'))
    pom.log_measurements(os.path.join(LOG_DIR, 'pom-measurements.log'))

    optitrack.set_logfile(os.path.join(LOG_DIR, 'opti.log'))

    rotorcraft.log(os.path.join(LOG_DIR, 'rotorcraft.log'))
    rotorcraft.start()

    nhfc.log(os.path.join(LOG_DIR, 'nhfc.log'))
    nhfc.set_current_position()

    time.sleep(0.5)

    rotorcraft.servo(ack=True)


def stop():
    rotorcraft.stop()
    rotorcraft.log_stop()

    nhfc.stop()
    nhfc.log_stop()

    pom.log_stop()
    optitrack.unset_logfile()


def simulation():
  setup()
  start()

  try:
    # initial hover at wp0
    time.sleep(2.0)

    waypoints = [
      (1.0,  1.0, 1.0, 0.0,         5.0),   # wp1
      (1.0, -1.0, 1.0, math.pi/2,   5.0),   # wp2
      (0.0,  0.0, 0.0, 0.0,         5.0),   # wp3
    ]

    for x, y, z, yaw, wait_time in waypoints:
      nhfc.set_position(x, y, z, yaw)
      time.sleep(wait_time)

  finally:
    stop()
    
if __name__ == "__main__":
    simulation()
