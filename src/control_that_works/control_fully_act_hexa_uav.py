import genomix
import os
import time
import math

g = genomix.connect()
g.rpath(os.environ['HOME'] + '/openrobots/lib/genom/pocolibs/plugins')

optitrack = g.load('optitrack')
rotorcraft = g.load('rotorcraft')
pom = g.load('pom')
uavpos = g.load('uavpos')
uavatt = g.load('uavatt')

LOG_DIR = os.path.join(os.environ['TK3LAB_WS'], 'logs', 'fahexa_uav')
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

    # rotorcraft reads commands from uavatt
    rotorcraft.connect_port({
        'local': 'rotor_input',
        'remote': 'uavatt/rotor_input'
    })

    # --- uavpos ---
    uavpos.set_mass({'mass': 2.72})

    uavpos.set_xyradius({'rxy': 2.0})   # from the slide guideline

    uavpos.set_saturation({'sat': {
        'x': 1,
        'v': 1,
        'ix': 0
    }})

    uavpos.set_servo_gain({'gain': {
        'Kpxy': 5.0,
        'Kpz': 5.0,
        'Kvxy': 6.0,
        'Kvz': 6.0,
        'Kixy': 0.0,
        'Kiz': 0.0
    }})

    uavpos.set_emerg({'emerg': {
        'descent': 0.1,
        'dx': 0.05,
        'dv': 0.2
    }})

    uavpos.connect_port({
        'local': 'state',
        'remote': 'pom/frame/robot'
    })

    # uavatt reads desired thrust/attitude from uavpos
    uavatt.connect_port({
        'local': 'uav_input',
        'remote': 'uavpos/uav_input'
    })

    # --- uavatt ---
    uavatt.set_gtmrp_geom({
        'rotors': 6,
        'cx': 0,
        'cy': 0,
        'cz': 0,
        'armlen': 0.38998,
        'mass': 2.72,
        'rx': 20,
        'ry': 0,
        'rz': -1,
        'cf': 2e-3,
        'ct': 1.9e-5
    })

    uavatt.set_mass({'mass': 2.72})

    uavatt.set_wlimit({
        'wmin': 0.0,
        'wmax': 120.0
    })


    uavatt.set_servo_gain({'gain': {
        'Kqxy': 4.0,
        'Kqz': 0.1,
        'Kwxy': 1.0,
        'Kwz': 0.1
    }})

    uavatt.set_emerg({'emerg': {
        'dq': 20,
        'dw': 20
    }})

    uavatt.connect_port({
        'local': 'rotor_measure',
        'remote': 'rotorcraft/rotor_measure'
    })

    uavatt.connect_port({
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

    pom.connect_port({
        'local': 'measure/mocap',
        'remote': 'optitrack/bodies/FH_4'
    })
    pom.add_measurement('mocap')


def start():
    pom.log_state('/tmp/pom.log')
    # pom.log_measurements('/tmp/pom_m.log')   # opzionale, non serve per far volare
    optitrack.set_logfile('/tmp/opti.log')

    rotorcraft.log('/tmp/rot.log')
    uavpos.log('/tmp/upos.log')
    uavatt.log('/tmp/uatt.log')

    rotorcraft.start()

    time.sleep(0.5)

    # hover iniziale
    uavpos.set_current_position()

    time.sleep(0.5)

    # QUESTO è il punto chiave
    uavatt.servo(ack=True)

    time.sleep(0.2)

    rotorcraft.servo(ack=True)


def stop():
    rotorcraft.stop()
    rotorcraft.log_stop()

    uavpos.stop()
    uavpos.log_stop()

    uavatt.stop()
    uavatt.log_stop()

    pom.log_stop()
    optitrack.unset_logfile()


def simulation():
    setup()
    start()

    try:
        time.sleep(2.0)

        waypoints = [
            (1.0,  1.0, 1.0, 0.0,       5.0),
            (1.0, -1.0, 1.0, math.pi/2, 5.0),
            (0.0,  0.0, 0.0, 0.0,       5.0),
        ]

        for x, y, z, yaw, wait_time in waypoints:
            uavpos.set_position(x, y, z, yaw)
            time.sleep(wait_time)

    finally:
        stop()


if __name__ == "__main__":
    simulation()
