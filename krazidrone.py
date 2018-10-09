import time
from enum import Enum
import numpy as np
from threading import Timer
from pynput import keyboard

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

class DroneDirection(Enum):
    UP = 0
    DOWN = 1
    FORWARD = 2
    LEFT = 3
    RIGHT = 4
    BACK = 5


class KraziDrone(Drone):
    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.current_position = np.array([0.0, 0.0, 0.0])
        self.default_altitude = 1.0

        # initial state
        self.flight_state = States.MANUAL

        # callbacks
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        #self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.MANUAL:
            pass
        elif self.flight_state == States.TAKEOFF:
            if -1.0*self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 0.2:
                pass
                #print("WAY POINT - GET THERE")
        elif self.flight_state == States.LANDING:
            pass
        elif self.flight_state == States.DISARMING:
            #print("DISARMING - GET THERE")
            pass
        elif self.flight_state == States.MANUAL:
            pass

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if abs(self.local_position[2] < 0.01):
                #self.manual_transition()
                self.disarming_transition()

    def state_callback(self):
        pass

    def takeoff_transition(self):
        print("takeoff transition")
        self.target_position[2] = self.default_altitude
        self.takeoff(self.target_position[2])
        self.flight_state = States.TAKEOFF

        #t = Timer(5.0, self.landing_transition)
        #t.start()

    def waypoint_transition(self):
        print("waypoint transition")
        self.flight_state = States.WAYPOINT

    def landing_transition(self):
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        print("disarm transition")
        self.disarm()
        self.release_control()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        print("manual transition")
        self.stop()
        self.flight_state = States.MANUAL

    def start(self):
        self.connection.start()

    def move_position(self, direction, distance):
        if direction == DroneDirection.FORWARD:
            [x, y, z] = [distance, 0.0, 0.0]
        elif direction == DroneDirection.LEFT:
            [x, y, z] = [0.0, -distance, 0.0]
            pass
        elif direction == DroneDirection.RIGHT:
            [x, y, z] = [0.0, distance, 0.0]
            pass
        elif direction == DroneDirection.BACK:
            [x, y, z] = [-distance, 0.0, 0.0]
            pass
        elif direction == DroneDirection.UP:
            [x, y, z] = [0.0, 0.0, distance]
            pass
        elif direction == DroneDirection.DOWN:
            [x, y, z] = [0.0, 0.0, -distance]
            pass
        self.target_position += [x, y, z]
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)

    def on_press(self, key):
        if key.char == 'u':
            if self.flight_state == States.MANUAL or self.flight_state == States.DISARMING:
                self.takeoff_transition()
            print('u')

        if key.char == 'w':
            if self.flight_state == States.WAYPOINT or self.flight_sate == States.TAKEOFF:
                self.move_position(DroneDirection.FORWARD, 1.0)
            print('w')

        if key.char == 's':
            #self.mc.start_back(self.velocity)
            if self.flight_state == States.WAYPOINT or self.flight_sate == States.TAKEOFF:
                self.move_position(DroneDirection.BACK, 1.0)
            print('s')

        if key.char == 'a':
            if self.flight_state == States.WAYPOINT or self.flight_sate == States.TAKEOFF:
                self.move_position(DroneDirection.LEFT, 1.0)
            print('a')

        if key.char == 'd':
            if self.flight_state == States.WAYPOINT or self.flight_sate == States.TAKEOFF:
                self.move_position(DroneDirection.RIGHT, 1.0)
            print('d')

        if key.char == 'r':
            if self.flight_state == States.WAYPOINT or self.flight_sate == States.TAKEOFF:
                self.move_position(DroneDirection.UP, 0.5)
            print('r')

        if key.char == 'f':
            if self.flight_state == States.WAYPOINT or self.flight_sate == States.TAKEOFF:
                self.move_position(DroneDirection.DOWN, 0.5)
            print('f')

        if key == keyboard.Key.space:
            #self.mc.start_up(self.velocity)
            print('space')

        if key.char == 'l':
            if self.flight_state == States.WAYPOINT or self.flight_state == States.TAKEOFF:
                self.landing_transition()

        if key.char == 'q':
            #self.mc.start_turn_left(self.ang_velocity)
            print('q')

        if key.char == 'e':
            #self.mc.start_turn_right(self.ang_velocity)
            print('e')

    def on_release(self, key):
        pass
        #self.mc.stop()
        #print(key.char + ' released')
        #:print("")


if __name__ == "__main__":
    conn = CrazyflieConnection('radio://0/80/250K')

    drone = KraziDrone(conn)
    time.sleep(2)
    drone.start()

    with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
        listener.join()