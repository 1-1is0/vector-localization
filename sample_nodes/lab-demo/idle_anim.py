#!/usr/bin/env python3

"""Idle animation that makes Vector turn and move forward random amounts.

Vector will stay within a range of its starting position.
It will also avoid obstacles by using the proximity sensor.
"""
import math
import random
import time
from threading import Event

import numpy
import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import Proximity, Drive, RobotStatus
from std_msgs.msg import Bool, Float32


ANGLE_MULTIPLIER = 120
BASE_SPEED = 50  # [mm/s]


class IdleAnimation:
    def __init__(self):
        self.motor_drive_pub = Publisher("/motors/wheels", Drive, queue_size=1)
        self.motor_stop_pub = Publisher("motors/stop", Bool, queue_size=1)
        self.lift_pub = Publisher("/behavior/lift_height", Float32, queue_size=1)
        self.proximity_sub = Subscriber("/proximity", Proximity, self.proxim_callback)
        self.prompt_pub = Publisher("/labdemo/prompt", Bool, queue_size=1)

        Subscriber("/labdemo/idle", Bool, self.hold_callback)

        self.hold = Event()
        self.coords = numpy.array((0.0, 0.0))
        self.theta = 0
        self.path_is_blocked = Event()

    def hold_callback(self, msg):
        if msg.data:
            # Received a signal saying to idle
            print("Start idle")
            self.hold.clear()
            self.path_is_blocked.clear()
            self.animate()
        else:
            print("Stop idle")
            self.hold.set()
            self.path_is_blocked.set()

            # Final failsafe due to race conditions
            time.sleep(0.2)
            self.motor_stop_pub.publish(True)

    def animate(self):
        while not self.hold.is_set():
            # turns a random angle
            self.prompt_pub.publish(True)
            turn_duration = random.random() + 0.6

            if self.hold.is_set():
                return
            self.motor_drive_pub.publish(BASE_SPEED, -BASE_SPEED, 0, 0)
            self.theta += turn_duration * ANGLE_MULTIPLIER
            self.hold.wait(turn_duration)
            self.motor_stop_pub.publish(True)

            self.hold.wait(random.random() * 3)

            # simulates where it will end up if it moves forward a random distance
            time_moved = random.random() + 1
            sim_dest = numpy.array((0.0, 0.0))
            sim_dest[0] = (
                self.coords[0] + math.cos(math.radians(self.theta)) * time_moved
            )
            sim_dest[1] = (
                self.coords[1] + math.sin(math.radians(self.theta)) * time_moved
            )
            sim_dist = numpy.linalg.norm(sim_dest)

            # vector only moves if it won't end up too far from the origin
            if sim_dist < 5.0 and not self.hold.is_set():
                self.lift_pub.publish(0.0)
                self.motor_drive_pub.publish(BASE_SPEED, BASE_SPEED, 0, 0)
                start_time = time.time()
                self.path_is_blocked.wait(time_moved)

                # if it stopped because an obstacle
                if self.path_is_blocked.is_set():
                    time_passed = time.time() - start_time
                    self.coords[0] += math.cos(math.radians(self.theta)) * time_passed
                    self.coords[1] += math.sin(math.radians(self.theta)) * time_passed
                else:
                    self.coords = sim_dest

                self.motor_stop_pub.publish(True)

                self.hold.wait(random.random() * 2)

    def proxim_callback(self, proxim):
        if proxim.distance < 50 and not proxim.is_lift_in_fov:
            self.path_is_blocked.set()
        else:
            self.path_is_blocked.clear()


if __name__ == "__main__":
    rospy.init_node("idle_anim")
    rospy.wait_for_message("/status", RobotStatus)
    IdleAnimation()
    rospy.spin()
