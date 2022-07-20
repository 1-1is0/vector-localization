#! /usr/bin/env python
import numpy as np
import rospy
import random
import time
import math

from rospy import Publisher, Subscriber
from anki_vector_ros.msg import RobotStatus
from anki_vector_ros.msg import Drive, Proximity, Pose
import smach_ros 
from numpy.random import uniform
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import scipy.stats as stats
from os.path import expanduser
import matplotlib.pyplot as plt
import map

#+++++++++++++++++++++ init vars ++++++++++++++++++++

x = 0.0
y = 0.0 
theta = 0.0
t_trans = 0
t_rot = 0
last_x=0
last_y=0

get_last_loc_flag = True
random_move_state = True
yaw_setpoint = 0
r_setpoint = 0
r_max_p_term = 0.5


laser_data = 0
new_laser_data_flag = False

particle_number = 500

laser_range = 0.04

loop_iter = 0

home = expanduser("~")
map_address = home + '/uni/robotics/anki_description_ws/src/anki_description/world/sample1.world'

is_halt = False
#------------------------------------------------------


#+++++++++++ read vector laser ranger finder +++++++++++

def laser_reader(msg):
    global new_laser_data_flag,laser_data

    new_laser_data_flag = True
    # laser_data = msg.range
    # print(msg)
    laser_data = msg.distance/1000
#------------------------------------------------------
 
# yaw_setpoint = uniform(-180, 180)
#r_setpoint = uniform(0,0.35)
def new_odometry(msg):
    global x
    global y
    global theta
    
    x = msg.x/1000
    y = msg.y/1000
 
    # rot_q = msg.angle
    # (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    # theta += 90*math.pi/180.0
    theta = msg.angle

#------------------------------------------------------

def particle_distance(particles):
    dist = np.zeros(len(particles))
    for i, particle in enumerate(particles):
        dist[i] = sum(np.sqrt((particle[0] - particles[:, 0])**2 + (particle[1] - particles[:, 1])**2))
    return particles[np.argmin(dist)]

#++++++++++++++++++ ROS init ++++++++++++++++++++++++++

rospy.init_node("particle_filter")

# sub = rospy.Subscriber("/odom", Odometry, new_odometry)
sub = rospy.Subscriber("/pose", Pose, new_odometry)

laser = rospy.Subscriber("/proximity", Proximity, callback=laser_reader, queue_size = 1)
pub = rospy.Publisher('/motors/wheels', Drive, queue_size = 1)

# pub = rospy.Publisher('/vector/cmd_vel', Twist, queue_size = 1)
# laser = rospy.Subscriber('/vector/laser', Range, laser_reader, queue_size = 1)
speed = Twist()

#------------------------------------------------------



#+++++++++++++++++++++ MAP ++++++++++++++++++++++++++++++
#   convert the 3d map to 2d map and get all points of every rectangle
# rects,global_map_pose,map_boundry = map.init_map(map_address)
rects,global_map_pose,map_boundry = map.init_map()
# print(map_boundry)
#   the center position of the map [x,y]
global_map_position = [float(global_map_pose[0]),float(global_map_pose[1])]

rects = map.add_offset(rects,global_map_position)
#   get the lines of every rectangle line defined by [start_point , end_point]
all_map_lines = map.convert_point_to_line(rects)

polygan = map.convert_to_poly(rects)

#--------------------------------------------------------

#+++++++++++++++++ init particles +++++++++++++++++++++++
particles = np.empty((particle_number, 3))
particles[:, 0] = uniform(map_boundry[0], map_boundry[1], size=particle_number) + global_map_position[0]
particles[:, 1] = uniform(map_boundry[2], map_boundry[3], size=particle_number) + global_map_position[1]
particles[:, 2] = np.random.choice([-90, 90, 180, 0], size=particle_number)*math.pi/180.0


for i in range(particle_number):
    while map.check_is_collition([particles[i][0],particles[i][1]] , polygan):
        # resample
        particles[i][0] = uniform(map_boundry[0], map_boundry[1]) + global_map_position[0]
        particles[i][1] = uniform(map_boundry[2], map_boundry[3]) + global_map_position[1]

#---------------------------------------------------------

while not rospy.is_shutdown():
    #   choose a random move for vector 
    #   which is acceptable for map
    yaw_setpoint = float(input("yaw_setpoint = "))
    
    r_setpoint = float(input("r_setpoint = "))

    # yaw_setpoint = random.choice([0,90,180,270])
    # r_setpoint = random.choice([0.15,0.25,0.35])
    print ('target = ', r_setpoint, yaw_setpoint)

    yaw_setpoint = yaw_setpoint * math.pi / 180.0
    def_rot = yaw_setpoint - theta
    
    # def_rot = int(input("def_rot = "))
    # def_rot = def_rot * math.pi / 180.0

    # yaw_setpoint = def_rot + theta
    # print ('target = ', r_setpoint, yaw_setpoint)

    loop_iter = 0

    robot_movment_complite_flag = False


    #++++++++++++++++++ move robot in gazebo ++++++++++++++++
    #   if the robot still moving just do this part
    
        
        # #   calculte the yaw error
        # yaw_error = yaw_setpoint - theta
        # speed.angular.z = yaw_error * 10

    # speed.linear.x = 0
    # speed.linear.y = 0
    # speed.linear.z = 0
    # speed.angular.x = 0
    # speed.angular.y = 0
    # speed.angular.z = 5

    
    
    # pub.publish(speed)
    # t_rot = def_rot / 5
    # rospy.sleep(  2.03 * t_rot )

    # #Forcing our robot to stop
    # speed.angular.z = 0
    # pub.publish(speed)

    pub.publish(20, -20, 0.0, 0.0)
    # t_rot = def_rot / 0.02
    print("theta", theta, "def rot", def_rot)
    t_rot = 3.75 * (np.rad2deg(def_rot)//90)
    print("DSF", t_rot)
    rospy.sleep(t_rot)
    pub.publish(0, 0, 0.0, 0.0)

    
    loop_iter += 1
    if loop_iter == 1:
        new_laser_data_flag = False 
        while not new_laser_data_flag:pass
        last_laser_data = laser_data
        print ('laser data after the rotation of vector', last_laser_data )
    
    print (f'check {last_laser_data} <=> {r_setpoint} + 0.03 ')
    if last_laser_data > r_setpoint + 0.03:
    # if last_laser_data > r_setpoint + 30:
        
        speed.linear.x = 0.05
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.x = 0
        speed.angular.y = 0
        speed.angular.z = 0
        print("i am moving")
        # pub.publihs(speed)
        pub.publish(50, 50, 0, 0)
        t_trans = r_setpoint / 0.05
        rospy.sleep(t_trans)
        print("t_trans", t_trans)
        speed.linear.x = 0
        # pub.publish(speed)
        pub.publish(0, 0, 0, 0)
        
        robot_movment_complite_flag = True
    else:
        robot_movment_complite_flag = True
    

            
#------------------------------------------------------------
#++++++++++++++++++++ predict (move) particles ++++++++++++++
#   if the robot in gazebo stoped then do rest of things
    if not is_halt:
        # print('move particle')

        # e_rot_in_rot = np.random.normal(def_rot, 0.0005, particle_number)
        # e_trans_in_rot = np.random.normal(0, 0.00075, particle_number)
        # total_rotation = e_rot_in_rot

        # total_transition = 0
        # if last_laser_data > r_setpoint + 0.03:
        #     e_trans_in_trans = np.random.normal(r_setpoint , 0.0036, particle_number)
        #     e_rot_in_trans = np.random.normal(0 , 0.0002, particle_number)
        #     total_transition = e_trans_in_trans + e_rot_in_trans
        #     total_rotation = e_rot_in_rot + e_trans_in_rot

        # particles[:, 0] += np.cos(yaw_setpoint) * total_transition
        # particles[:, 1] += np.sin(yaw_setpoint) * total_transition
        # particles[:, 2] += total_rotation

        # transition
        v=0.05
        vhat= v + np.random.normal(0, 0.0027)
        what = 0 + np.random.normal(0.000125, 0.002)
        if what == 0:
            what = 0.00000001
        gamahat=0
        particles[:, 0] +=  (-1 * (vhat/what) * np.sin(yaw_setpoint) ) + (vhat/what) * np.sin(yaw_setpoint+what*t_trans)
        particles[:, 1] +=  ( (vhat/what) * np.cos(yaw_setpoint) ) + (-1 * vhat/what) * np.cos(yaw_setpoint+what*t_trans)
        particles[:, 2] += what*t_trans 


        # rotation
        v=0
        vhat= v + np.random.normal(0, 0.0027)
        what = 0 + np.random.normal(0.000125, 0.002)
        if what == 0:
            what = 0.00000001
        gamahat=5 + np.random.normal(0, 0.0055)
        particles[:, 0] +=  (-1 * (vhat/what) * np.sin(yaw_setpoint) ) + (vhat/what) * np.sin(yaw_setpoint+what*t_rot)
        particles[:, 1] +=  ( (vhat/what) * np.cos(yaw_setpoint) ) + (-1 * (vhat/what) * np.cos(yaw_setpoint+what*t_rot))
        particles[:, 2] += what*t_rot + gamahat * t_rot





        for i in range(particle_number):
            if map.check_is_collition([particles[i][0],particles[i][1]] , polygan) or \
            map.out_of_range(particles[i],global_map_position,map_boundry):
                particles[i][0] = -10

        #-------------------------------------------------------------

        #++++++++++++++++++++ sensor update ++++++++++++++++++++++++++
        weights = np.zeros(particle_number) + 1e-8
        
        plt.clf()
        # reverse y axis
        plt.gca().invert_yaxis()
        # plot map
        map.plot_map(all_map_lines)
        
        # wait for new laser data
        new_laser_data_flag = False 
        while not new_laser_data_flag:pass
        last_laser_data = laser_data
        print ('new laser data',last_laser_data)

        #   do for all particles
        for i in range(particle_number):

            #   skip trash particles
            if particles[i][0] == -10:
                weights[i] = 0
                continue

            #   calculate the start and the end of sensor line (the length is 0.4)
            #   [start_point , end_point]
            sensor_line = [ [particles[i][0], particles[i][1]] , \
                [ 0.4*math.cos(particles[i][2])+particles[i][0] , \
                0.4*math.sin(particles[i][2])+particles[i][1] ] ]
            #plt.plot( [sensor_line[0][1], sensor_line[1][1]] , [sensor_line[0][0], sensor_line[1][0]])
            
            min_distance = 10
            collission_point = False
            
            for line in all_map_lines:
                #   calculate the intersection point
                intersection_point = map.find_intersection(line[0], line[1] , sensor_line[0], sensor_line[1])

                #   check for the existance of intersection point 
                if intersection_point != False:
                    #   calculate the distance of intersection point and particle position
                    distance = math.sqrt((particles[i][0]-intersection_point[0])**2 + (particles[i][1]-intersection_point[1])**2 )
                    if min_distance >= distance:
                        min_distance = distance
                        collission_point = intersection_point

            #   sensor hit situation
            if collission_point != False:
                #   particle sensor line [ start_point , end_point ]
                plt.plot( [particles[i][1], collission_point[1]], [particles[i][0], collission_point[0]] ,\
                    color='blue', linestyle=':')
                particle_sensor_line = [ [ particles[i][0] , particles[i][1] ], collission_point ]
            #   sensor max situation
            else:
                #   the sensor line is the full lenth sensor line (0.4)
                particle_sensor_line = sensor_line
                min_distance = 0.4


            #   sensor model is a normal distribution [mean,var]
            #   mean is the distance that particles read 
            #   var is 0.000097
            # print i, min_distance
            weights[i] = stats.norm(min_distance, 0.0097).pdf(last_laser_data)
            
        # print weights
        #   plotting every particle position and orientation
        for i, particle in enumerate(particles):
            # print 'particle position = ', particle[0], particle[1], particle[2]*180.0/math.pi
            # plt.text(particle[1]+0.01, particle[0]+0.01, '{}'.format(i))
            plt.arrow(particle[1], particle[0], 0.00001*math.sin(particle[2]), \
                0.00001*math.cos(particle[2]), head_width = 0.02, fill=False, overhang = 0.6)
        
        # plot robot position
        plt.arrow(y, x, 0.00001*math.sin(theta), 0.00001*math.cos(theta), \
            color = 'red', head_width = 0.02, overhang = 0.6)
        plt.plot( [y, y+0.4*math.sin(theta)], [x, x+0.4*math.cos(theta)], color='red')
        # print 'robot position=', x, y, theta*math.pi/180
        
        #   normalize the weights
        weights /= weights.sum()
        #-------------------------------------------------------------    

        #+++++++++++++++++++++++ resample +++++++++++++++++++++++++++++
        
        indices_of_most_valuable_particles = (-weights).argsort()[:int(0.3 * particle_number)]
        # indexes = np.random.choice(indices, indices, p=weights[indices])
        
        estimate = particle_distance(particles[indices_of_most_valuable_particles])
        plt.arrow(estimate[1], estimate[0], 0.00001*math.sin(estimate[2]), 0.00001*math.cos(estimate[2]), \
            color = 'blue', head_width = 0.02, overhang = 0.6)
        
        # if you want to plot episodic change 
        # plt.draw() to plt.show()
        plt.draw()
        plt.pause(0.2)
        
        around_estimate_x = particles[np.where(abs(particles[:, 0] - estimate[0]) <= 0.05)]
        around_estimate_y = around_estimate_x[np.where(abs(around_estimate_x[:, 1] - estimate[1]) <= 0.05)]
        if len(around_estimate_y) >= 0.25*particle_number:
            plt.savefig('ending position.png')
            print ('estimate position =', estimate[0], estimate[1], estimate[2]*math.pi/180)
            print ('robot position=', x, y, theta*math.pi/180)
            is_halt = True
            break              
        
        random_size = particle_number - len(indices_of_most_valuable_particles)
        random_around_valuable_size = int(0.5*random_size)

        most_valuable_particles = particles[indices_of_most_valuable_particles]
        random_particles_around_most_valuables = np.empty((random_around_valuable_size, 3))
        random_particles_kidnapping = np.empty(((random_size - random_around_valuable_size), 3))
        

        random_particles_kidnapping[:, 0] = uniform(map_boundry[0], map_boundry[1], size= \
                                                    (random_size - random_around_valuable_size)) + global_map_position[0]
        random_particles_kidnapping[:, 1] = uniform(map_boundry[2], map_boundry[3], size= \
                                                    (random_size - random_around_valuable_size)) + global_map_position[1]
        random_particles_kidnapping[:, 2] = np.random.choice([-90, 90, 180, 0], size= \
                                                                (random_size - random_around_valuable_size))*math.pi/180.0

        for i in range(len(random_particles_kidnapping)):
            while map.check_is_collition([random_particles_kidnapping[i][0],random_particles_kidnapping[i][1]] , polygan):
                # resample
                random_particles_kidnapping[i][0] = uniform(map_boundry[0], map_boundry[1]) + global_map_position[0]
                random_particles_kidnapping[i][1] = uniform(map_boundry[2], map_boundry[3]) + global_map_position[1]

        
        indices_of_random_valuable_particles = np.random.choice(most_valuable_particles.shape[0], random_around_valuable_size)  
        
        random_particles_around_most_valuables[:, 0] = uniform(-0.05, 0.05, size=(random_around_valuable_size)) 
        random_particles_around_most_valuables[:, 1] = uniform(-0.05, 0.05, size=(random_around_valuable_size)) 
        random_particles_around_most_valuables[:, 2] = 0

        random_particles_around_most_valuables += most_valuable_particles[indices_of_random_valuable_particles]
        
        '''
        # actual roulette wheel resampling with 20% random sampling
        indexes = np.random.choice(particle_number, int(0.8*particle_number), p=weights)
        
        estimate = particle_distance(particles)
        plt.arrow(estimate[1], estimate[0], 0.00001*math.sin(estimate[2]), 0.00001*math.cos(estimate[2]), \
            color = 'blue', head_width = 0.02, overhang = 0.6)
        
        # if you want to plot episodic change 
        # plt.draw() to plt.show()
        plt.draw()
        plt.pause(0.2)
        
        around_estimate_x = particles[np.where(abs(particles[:, 0] - estimate[0]) <= 0.05)]
        around_estimate_y = around_estimate_x[np.where(abs(around_estimate_x[:, 1] - estimate[1]) <= 0.05)]
        if len(around_estimate_y) >= 0.15*particle_number:
            plt.savefig('ending position.png')
            print 'estimate position =', estimate[0], estimate[1], estimate[2]*math.pi/180
            print 'robot position=', x, y, theta*math.pi/180
            is_halt = True
            break              
        
        random_size = particle_number - int(0.8*particle_number)
        random_particles_kidnapping = np.empty((random_size, 3))
        
        random_particles_kidnapping[:, 0] = uniform(map_boundry[0], map_boundry[1], size=random_size) + global_map_position[0]
        random_particles_kidnapping[:, 1] = uniform(map_boundry[2], map_boundry[3], size=random_size) + global_map_position[1]
        random_particles_kidnapping[:, 2] = np.random.choice([-90, 90, 180, 0], size=random_size)*math.pi/180.0

        for i in range(len(random_particles_kidnapping)):
            while map.check_is_collition([random_particles_kidnapping[i][0],random_particles_kidnapping[i][1]] , polygan):
                # resample
                random_particles_kidnapping[i][0] = uniform(map_boundry[0], map_boundry[1]) + global_map_position[0]
                random_particles_kidnapping[i][1] = uniform(map_boundry[2], map_boundry[3]) + global_map_position[1]

        # update particles
        particles = np.concatenate((particles[indexes], random_particles_kidnapping))
        '''
        # update particles
        particles = np.concatenate((most_valuable_particles, random_particles_kidnapping, random_particles_around_most_valuables))
        #--------------------------------------------------------------
        
        # prepare for move randomly again 
        random_move_state = True
