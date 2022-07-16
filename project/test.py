#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Robot import Robot
#from RobotDummy import RobotDummy
from World import World
from ParticleFilter import ParticleFilter

import rospy
import time
import random
from math import pi

forward_noise_dict = {
    # cm : speed mean std
    50: [87.76-100, 7.587745221804723],
    100: [91.16-100, 3.5763394807585946],
    150: [92.14-100, 3.162342195788638],
}
turn_noise_dict = {
    # degree: degree mean std
    90: [85.0-90, 2.1081851067789197],
}

sensor_noise_dict = {
    50: [54.77891036906854, 3.1418601224248195],
    100: [109.82300581992469, 3.079733466101635],
    200: [216.16912487708947, 2.0128058519994476],
    300: [320.3087800065768, 4.12825053641244],
    370: [387.51603104961185, 4.162953090683025]
}

def main():
    print("**** Start localization ****")
    time_start = time.time()
    nParticles = 1000
    
    length= 20 
    width= 20
    landmarks = [[-10.0, -10.0], [10.0, 10.0], [-10.0, 10.0], [10.0, -10.0], [0, 10]]

    world = World(length=20, width=20, landmarks=landmarks)

    particleFilter = ParticleFilter(nParticles, forward_noise_dict[50], turn_noise_dict[90], sensor_noise_dict, world)
    rospy.init_node("main_localization", anonymous=True)
    myRobot = Robot(forward_noise_dict[50], turn_noise_dict[90], sensor_noise_dict, world)

    # set robot
    #myRobot = RobotDummy(world)
    myRobot.prettyPrint()
    #TODO set velocities
    omega=0.5
    turn1 = pi/2
    turn2 = -pi/2
    sp=0.2
    dist=5
    steps= 100

    #  myRobot.rotate(0.5, pi)
    particleFilter.allParticleDistance()       
    # getting timing
    time_end = time.time()
    running_time = time_end - time_start
    print("Program run %.4f seconds for %s steps and %s particles"%(running_time, steps, particleFilter.n))
            
    
if __name__ == "__main__":
    main()
