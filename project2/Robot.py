import rospy
from rospy import Publisher, Subscriber
from time import sleep
# for gazebo sim
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
# for real robot
from anki_vector_ros.msg import Drive, Proximity

from util import *
from math import *
import random

class Robot:
    n_lazer_sample = 10

    motor_pub = "/vector/cmd_vel"
    lazer_sub = "/vector/laser"

    #  motor_pub = "/motors/wheels"
    #  lazer_sub = "/proximity",

    def __init__(self, forward_noise, turn_noise,sense_noise, world):
        rospy.loginfo("Initialize robot")
        self.world = world       
        # noise for the estimation
        self.sense_noise = sense_noise
        self.forward_noise=forward_noise
        self.turn_noise=turn_noise
        self.x=0
        self.y=0
        self.orientation=0


    def prettyPrint(self):
        print(f"Estimate:\t({self.x:.2f}, {self.y:.2f}) \t Orientation: {self.orientation:.2f}")
        #print("Truth: \t\t(%.2f, %.2f) \t Orientation: %.2f"%(self.trueX, self.trueY, self.trueOrientation))

    def proximity_call_back(self, msg):
        print("####### msg", msg)
        self.lazer_distance = msg.distance


    def sense(self):
        rospy.loginfo("Sensing the distance to the wall")
        #TODO distance=عدد فاصله ای که سنسور میخواند
        # return distance
        #  sub = Subscriber(self.lazer_sub, Proximity, callback=self.proximity_call_back)
        self.lazer_distance = 0
        for i in range(self.n_lazer_sample):
            sample = rospy.wait_for_message(self.lazer_sub, Range)
            self.lazer_distance += sample.range * 1000 # convert to milimeter
            rospy.sleep(0.1)
        self.lazer_distance /= self.n_lazer_sample

        print(self.lazer_distance)
        if (self.lazer_distance < 100):
            key = 50
        elif (100 < self.lazer_distance < 200):
            key = 100
        elif (200 < self.lazer_distance < 300):
            key = 200
        elif (300 < self.lazer_distance < 370):
            key = 300
        else:
            key = 370       
        mean, std = self.sense_noise[key]
        # val = self.lazer_distance - np.random.normal(abs(mean-key), std)
        val = self.lazer_distance
        print("final sens value:", val)
        return val


    def go(self, speed, distance):
        print("Go forward: speed=%s, distance=%s"%(speed, distance))        
        velocity_publisher = rospy.Publisher(self.motor_pub, Twist, queue_size=1000)
        # set movement
        vel_msg = VelMsg()
        #TODO TODO velocity noise
        v=speed
        if ( distance <= 80):
            key = 50
        else:
            key = 100
        mean, std=self.forward_noise[key]
        #TODO check the correctness of speed noise
        # speed=speed+np.random.normal(abs(-mean+v),std)
        speed=speed
        #for real robot we have: delta_t=desired_dist/speed
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
            #TODO اگه لازم شد عدد 1.5 نوشته شده در زیر تغییر کند
            rospy.sleep(1.5)
            break

    def rotate(self, omega, turn):
        print(f"Rotate: omega={omega}, turn={turn:0.3f}")
        velocity_publisher = rospy.Publisher(self.motor_pub, Twist, queue_size=1000)
        # set movement
        vel_msg = VelMsg()
        #TODO omega noise
        # for real robot we have: true_turn=turn+np.random.normal(abs(turn-mean),std);mean,std for turn data
        # we have v and true_turn so we calculate the time
        mean,std=self.turn_noise
        turn=turn+np.random.normal(abs(turn-mean),std)
        print('final turn', turn)
        t_rot = turn / omega
        rospy.sleep(2.03 * t_rot)
        # if (turn < 0):
        #     # turn = abs(turn)
        #     vel_msg.setTurn(-omega)
        # else:
        #     vel_msg.setTurn(omega)
        # while not rospy.is_shutdown():
        #     # Setting current time for distance calculus
        #     t0 = rospy.Time.now().to_sec()
        #     # guarantee that it is non-zero
        #     while (not t0 > 0):
        #         t0 = rospy.Time.now().to_sec()
        #     current_turn = 0
        #     # loop to move in a specified distance
        #     while(current_turn < turn):
        #         #  print("cure", current_turn)
        #         velocity_publisher.publish(vel_msg.msg)
        #         t1 = rospy.Time.now().to_sec()
        #         current_turn = omega * (t1-t0)
                #print("Turn: %.3f | Speed: %.3f | Time: %.3f"%(current_turn, speed, (t1-t0)))
            # stop robot
        print("Turning finished...")
        vel_msg.setTurn(0)
        velocity_publisher.publish(vel_msg.msg)
        rospy.sleep(1.5)

