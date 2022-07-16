#!/usr/bin/env python0
"""Sample program to make Vector move."""

from fileinput import filename
from time import sleep
# from json

import rospy
from rospy import Publisher, Subscriber
from anki_vector_ros.msg import RobotStatus
from anki_vector_ros.msg import Drive, Proximity
import smach_ros

sample_data = []
def print_proximity_call_back(msg):
    global sample_data
    # print("msg", msg)
    sample_data.append(str(msg))
    print(len(sample_data), msg.distance)



def main():
    print("Setting up publishers")
    # move_pub = Publisher("/motors/wheels", Drive, queue_size=1)
    sub = Subscriber("/proximity", Proximity, callback=print_proximity_call_back)

    sleep(125.0)
    # sleep(5.0)
    print('end')

    
    # open file in write mode
    cm = 37

    # env = "normal"
    # env = "light"
    # env = "dark"
    env = "shadow"
    filename = rf"sample_data_{env}_{cm}cm.txt"
    with open(filename, 'w') as fp:
        for data in sample_data:
            # write each item on a new line
            fp.write("%s\n" % data)
        print('Done')



if __name__ == "__main__":
    print("satrt")
    rospy.init_node("vector_hello_world")
    rospy.wait_for_message("/status", RobotStatus)
    print("at the end")

    main()

    # print(sample_data)