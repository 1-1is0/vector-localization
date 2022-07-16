
from turtle import distance
from sklearn.mixture import GaussianMixture
from World import World
import rospy
from rospy import Publisher, Subscriber
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from anki_vector_ros.msg import Drive, Proximity

from util import *
from math import *
import random
from ParticleFilter import ParticleFilter
from time import sleep
from sklearn.cluster import DBSCAN, MeanShift
# https://scikit-learn.org/stable/modules/clustering.html

size = 20
landmarks = [[-10.0, -10.0], [10.0, 10.0], [-10.0, 10.0], [10.0, -10.0], [0, 10]]
world = World(20, 20, landmarks)

# set particle filter
#TODO set variances
particleFilter = ParticleFilter(100, 0.05, 0.05, world)
particleFilter.particles
particleFilter.visualize(step=1, old_particles=particleFilter.particles, estimated_position=None)

motor_pub = "/vector/cmd_vel"
lazer_sub = "/vector/laser"
rospy.init_node("vector_particle_filter")

from sensor_msgs.msg import Range
for j in range(10):
    i = rospy.wait_for_message(lazer_sub, Range)
    print("I is", i.range*1000)
    sleep(0.5)

# velocity_publisher = rospy.Publisher(motor_pub, Drive, queue_size=1)
# t = 5
# sleep(0.5)
# velocity_publisher.publish(speed, speed, 0.0, 0.0)
# sleep(t)
# velocity_publisher.publish(0.0, 0.0, 0.0, 0.0)
# sleep(2)
from geometry_msgs.msg import Twist
class VelMsg:
    def __init__(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        # vel_msg.linear.y = 0
        # vel_msg.linear.z = 0
        # vel_msg.angular.x = 0
        # vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.msg = vel_msg

    def setForward(self, speed):
        self.msg.linear.x = speed
    
    def setTurn(self, speed):
        self.msg.angular.z = speed

def move():
    speed = 0.2
    distance = 1
    print("Go forward: speed=%s, distance=%s"%(speed, distance))
    velocity_publisher = rospy.Publisher(motor_pub, Twist, queue_size=1000)

    # set movement
    vel_msg = VelMsg()
    vel_msg.setForward(speed)

    while not rospy.is_shutdown():
        # Setting current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        # guarantee that it is non-zero
        while (not t0 > 0):
            t0 = rospy.Time.now().to_sec()
        current_distance = 0
        # loop to move in a specified distance
        while (current_distance < distance):
            # publish velocity
            velocity_publisher.publish(vel_msg.msg)
            # take actual time to velocity calculus
            t1 = rospy.Time.now().to_sec()
            # calculate distance of the pose
            current_distance = speed * (t1-t0)

        # stop robot
        print("Moving forward finished...")
        vel_msg.setForward(0)
        velocity_publisher.publish(vel_msg.msg)
        rospy.sleep(1.5)
        break

