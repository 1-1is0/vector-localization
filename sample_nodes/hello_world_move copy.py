#!/usr/bin/env python3
"""Sample program to make Vector move."""

from time import sleep
# from json

import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import RobotStatus
from anki_vector_ros.msg import Drive, Proximity, 
import smach_ros

sample_data = []
def print_proximity_call_back(msg):
    global sample_data
    print("msg", msg)
    sample_data.append((len(sample_data) + 1, str(msg)))



def main():
    print("Setting up publishers")

    move_pub = Publisher("/behavior/lift_height", Drive, queue_size=1)
    # sub = Subscriber("/proximity", Proximity, callback=print_proximity_call_back)

    


    # Need small delay to setup publishers
    sleep(0.5)

    print("Executing commands")
    move_pub.publish(-100.0, -100.0, 0.0, 0.0)
    sleep(3.0)
    move_pub.publish(0.0, 0.0, 0.0, 0.0)


if __name__ == "__main__":
    print("satrt")
    rospy.init_node("vector_hello_world")
    rospy.wait_for_message("/status", RobotStatus)
    print("at the end")

    main()

    print(sample_data)