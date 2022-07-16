#!/usr/bin/env python3
# first 10 with -20
# rest 5 with +20
"""Sample program to make Vector move."""
from math import pi
from time import sleep
from pyrsistent import s

from sqlalchemy import Float
# from json

import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import RobotStatus
from anki_vector_ros.msg import Drive, Proximity, Pose
import smach_ros

sample_data = []
def print_proximity_call_back(msg):
    # print(msg)
    move_pub = Publisher("/motors/wheels", Drive, queue_size=1)
    rad = msg.angle
    angel = rad * 180 / pi
    print("rad", rad)
    # 20 degree
    # 20, 30 , 45, 90
    # limit = 0.349
    limit = 90
    # 90 degree
    # limit = 90

    limit = limit * pi / 180
    if -rad > limit:
        move_pub.publish(0.0, 0.0, 0.0, 0.0)
        print("#stop in if")
        print("angle", angel)



def main():
    print("Setting up publishers")
    move_pub = Publisher("/motors/wheels", Drive, queue_size=1)

    # head_pub = Publisher("/motors/Head", float, queue_size=1)
# /motors/head
    sub = Subscriber("/pose", Pose, callback=print_proximity_call_back)


    # Need small delay to setup publishers
    sleep(0.5)
    v = 20.0
    # 5
    # t = 0.5
    # 10
    # t = 1
    # 15
    # t = 1.5
    # 20
    # t = 2
    t = 5
    print("Executing commands")
    move_pub.publish(v, -v, 0.0, 0.0)
    sleep(t)
    move_pub.publish(0.0, 0.0, 0.0, 0.0)

    sleep(2)
    # h = 20
    # head_pub.publish(h)
    # sleep(1)
    # head_pub.publish(-h)
    # sub.


if __name__ == "__main__":
    print("satrt")
    rospy.init_node("vector_hello_world")
    rospy.wait_for_message("/status", RobotStatus)
    print("at the end")

    main()

    print(sample_data)