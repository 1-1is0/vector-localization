#!/usr/bin/env python3
from rospy import Subscriber
from std_msgs.msg import Float32, Bool
from anki_vector_ros.msg import Drive


class Motors:
    def __init__(self, robot):
        self.robot = robot

        self.head_motor_sub = Subscriber("/motors/head", Float32, self.set_head_motor)
        self.lift_motor_sub = Subscriber("/motors/lift", Float32, self.set_lift_motor)
        self.wheel_motors_sub = Subscriber(
            "/motors/wheels", Drive, self.set_wheel_motors
        )
        self.stop_motors_sub = Subscriber("/motors/stop", Bool, self.stop_motors)

    def set_head_motor(self, speed):
        self.robot.motors.set_head_motor(speed.data)

    def set_lift_motor(self, speed):
        self.robot.motors.set_lift_motor(speed.data)

    def set_wheel_motors(self, speeds):
        if speeds.left_acc == 0 and speeds.right_acc == 0:
            self.robot.motors.set_wheel_motors(speeds.left, speeds.right)
        else:
            self.robot.motors.set_wheel_motors(
                speeds.left, speeds.right, speeds.left_acc, speeds.right_acc
            )

    def stop_motors(self, stop):
        if stop.data:
            self.robot.motors.stop_all_motors()
