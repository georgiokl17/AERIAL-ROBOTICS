import genomix
import os
import time
import math

# -------------------------
# CONNECT
# -------------------------
g = genomix.connect()
g.rpath(os.environ['HOME'] + '/openrobots/lib/genom/pocolibs/plugins')

optitrack = g.load('optitrack')
rotorcraft = g.load('rotorcraft')
pom = g.load('pom')
uavpos = g.load('uavpos')
uavatt = g.load('uavatt')
maneuver = g.load('maneuver')

LOG_DIR = os.path.join(os.environ['TK3LAB_WS'], 'logs', 'fah_traj')
os.makedirs(LOG_DIR, exist_ok=True)

# -------------------------
# SETUP
# -------------------------
def setup():

    # --- optitrack ---
    optitrack.connect({
        'host': 'localhost',
        'host_port': '1509',
        'mcast': '',
        'mcast_port': '0'
    })

    # --- rotorcraft ---
    rotorcraft.connect({'serial': '/tmp/pty-fahexa', 'baud': 0})

    rotorcraft.set_sensor_rate({'rate': {
        'imu': 1000,
        'mag': 0,
        'motor': 20,
        'battery': 1
    }})

    rotorcraft.set_imu_filter({
        'gfc': [20,20,20],
        'afc': [5,5,5],
        'mfc': [20,20,20]
    })

    # -------------------------
    # CONNECTIONS
    # -------------------------

    # rotorcraft ← uavatt
    rotorcraft.connect_port({
        'local': 'rotor_input',
        'remote': 'uavatt/rotor_input'
    })

    # uavatt ← sensors
    uavatt.connect_port({
        'local': 'rotor_measure',
        'remote': 'rotorcraft/rotor_measure'
    })

    uavatt.connect_port({
        'local': 'state',
        'remote': 'pom/frame/robot'
    })

    # uavpos ← maneuver
    uavpos.connect_port({
        'local': 'reference',
        'remote': 'maneuver/desired'
    })

    uavpos.connect_port({
        'local': 'state',
        'remote': 'pom/frame/robot'
    })

    # maneuver ← state
    maneuver.connect_port({
        'local': 'state',
        'remote': 'pom/frame/robot'
    })

    # uavatt ← uavpos
    uavatt.connect_port({
        'local': 'uav_input',
        'remote': 'uavpos/uav_input'
    })

    # -------------------------
    # UAVPOS CONFIG
    # -------------------------
    uavpos.set_mass({'mass': 2.72})

    uavpos.set_xyradius({'rxy': 2.0})

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

    # -------------------------
    # UAVATT CONFIG
    # -------------------------
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

    # -------------------------
    # POM
    # -------------------------
    pom.set_prediction_model('::pom::constant_acceleration')

    pom.connect_port({'local': 'measure/imu', 'remote': 'rotorcraft/imu'})
    pom.add_measurement('imu')

    pom.connect_port({'local': 'measure/mag', 'remote': 'rotorcraft/mag'})
    pom.add_measurement('mag')

    pom.connect_port({
        'local': 'measure/mocap',
        'remote': 'optitrack/bodies/FH'
    })
    pom.add_measurement('mocap')

# -------------------------
# START
# -------------------------
def start():

    pom.log_state(os.path.join(LOG_DIR, 'pom.log'))
    optitrack.set_logfile(os.path.join(LOG_DIR, 'optitrack.log'))
    rotorcraft.log(os.path.join(LOG_DIR, 'rotorcraft.log'))
    uavpos.log(os.path.join(LOG_DIR, 'uavpos.log'))
    uavatt.log(os.path.join(LOG_DIR, 'uavatt.log'))
    maneuver.log(os.path.join(LOG_DIR, 'maneuver.log'))

    rotorcraft.start()
    time.sleep(1)

    uavpos.set_current_position()
    time.sleep(0.5)

    maneuver.set_current_state()
    time.sleep(0.5)

    # 🔥 QUESTA RIGA MANCAVA
    uavpos.servo(ack=True)

    # controller chain
    uavatt.servo(ack=True)
    rotorcraft.servo(ack=True)

    print(">>> SYSTEM READY")
# -------------------------
# STOP
# -------------------------
def stop():
    rotorcraft.stop()
    rotorcraft.log_stop()

    uavpos.stop()
    uavpos.log_stop()

    uavatt.stop()
    uavatt.log_stop()

    maneuver.stop()
    maneuver.log_stop()

    pom.log_stop()
    optitrack.unset_logfile()

# -------------------------
# SIMULATION
# -------------------------
def simulation():
    setup()
    start()

    try:
        time.sleep(2)

        print(">>> TAKE OFF")

        # inizializzazione (importante)
        maneuver.set_current_state()

        # decollo
        maneuver.take_off(1.0, 5)
        time.sleep(6)

        print(">>> WAYPOINTS")

        waypoints = [
            (1.0,  1.0, 1.0, 0.0,         5.0),
            (1.0, -1.0, 1.0, math.pi/2,   5.0),
            (0.0,  0.0, 0.0, 0.0,         5.0),
        ]

        for x, y, z, yaw, duration in waypoints:
            print(f">>> goto {x},{y},{z}")
            maneuver.goto(x, y, z, yaw, duration)
            time.sleep(duration + 1)

    finally:
        stop()
# -------------------------
if __name__ == "__main__":
    simulation()
