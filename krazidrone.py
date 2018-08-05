import time
from enum import Enum
import numpy as np
from threading import Timer

from udacidrone import Drone
from udacidrone.messaging import MsgID
from udacidrone.connection import CrazyflieConnection

class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5
# KraziDrone is based with CrazyFlie 2.0
# States => MANUAL -> TAKEOFF -> WAYPOINT -> LANDING -> MANUAL


class KraziDrone(Drone):
    def __init__(self, connection):
        super().__init__(connection)

        # initial state
        self.flight_state = States.MANUAL

        # callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        #self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.MANUAL:
            self.takeoff_transition()
        elif self.flight_state == States.TAKEOFF:
            pass
        elif self.flight_state == States.WAYPOINT:
            pass
        elif self.flight_state == States.LANDING:
            pass
        elif self.flight_state == States.MANUAL:
            pass

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if abs(self.local_position[2] < 0.01):
                self.manual_transition()

    def state_callback(self):
        pass

    def takeoff_transition(self):
        print("takeoff transition")
        target_altitude = 1.0
        self.target_position[2] = target_altitude
        self.takeoff(target_altitude)
        self.flight_state = States.TAKEOFF

        t = Timer(5.0, self.landing_transition)
        t.start()

    def waypoint_transition(self):
        print("waypoint transition")
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.flight_state = States.MANUAL

    def start(self):
        self.connection.start()

if __name__ == "__main__":
    conn = CrazyflieConnection('radio://0/80/250K')

    drone = KraziDrone(conn)
    time.sleep(2)
    drone.start()