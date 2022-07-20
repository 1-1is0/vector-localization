from Robot import Robot
#from RobotDummy import RobotDummy
from World import World
from ParticleFilter import ParticleFilter
import numpy as np
import rospy
import time
import random
from math import pi
# forward_noise_dict = {
#     # cm : speed mean std
#     50: [87.76, 7.587745221804723],
#     100: [91.16, 3.5763394807585946],
#     150: [92.14, 3.162342195788638],
# }
# turn_noise_dict = {
#     # degree: degree mean std
#     # old whas 85
#     90: [np.deg2rad(85.0), np.deg2rad(2.1081851067789197)],
# }


# sensor_noise_dict = {
#     50: [54.77891036906854, 3.1418601224248195],
#     100: [109.82300581992469, 3.079733466101635],
#     200: [216.16912487708947, 2.0128058519994476],
#     300: [320.3087800065768, 4.12825053641244],
#     370: [387.51603104961185, 4.162953090683025]
# }
forward_noise_dict = {
    # cm : speed mean std
    50: [0, 7.587745221804723],
    100: [0, 3.5763394807585946],
    150: [0, 3.162342195788638],
}
turn_noise_dict = {
    # degree: degree mean std
    # old whas 85
    90: [np.deg2rad(85.0), np.deg2rad(2.1081851067789197)],
}


sensor_noise_dict = {
    50: [54.77891036906854, 3.1418601224248195],
    100: [109.82300581992469, 3.079733466101635],
    200: [216.16912487708947, 2.0128058519994476],
    300: [320.3087800065768, 4.12825053641244],
    370: [387.51603104961185, 4.162953090683025]
}
if __name__ == "__main__":
    """
    1. distribute particles
    2. move robot
    3. sense distance
    4. move particles
    5. calculate weights
    6. resample
    7. plot
        - resampled particles
        - old particles
    """
    print("**** Start localization ****")
    # start timing
    time_start = time.time()
    # choose number of particles
    nParticles = 1000
    # set world
    length= 900
    width =  900
    landmarks = [[-10.0, -10.0], [10.0, 10.0], [-10.0, 10.0], [10.0, -10.0], [0, 10]]
    world = World(length=length, width=width, landmarks=landmarks)
    # set particle filter
    #TODO set variances
    particleFilter = ParticleFilter(nParticles, forward_noise_dict[50], turn_noise_dict[90], sensor_noise_dict, world)
    # print("World size: %s"%particleFilter.WORLD.size)

    # start new node
    rospy.init_node("main_localization", anonymous=True)
    # initialize robot
    #TODO واریانس خطاهای مختلف ربات
    myRobot = Robot(forward_noise_dict, turn_noise_dict[90], sensor_noise_dict, world)

    # set robot
    myRobot.prettyPrint()
    #TODO set velocities
    omega=0.5
    turn1 = pi/2
    turn2 = -pi/2
    sp=0.02
    dist=5
    steps= 100
    move_condition = 100
  
    condition=True
    t = 0
    while condition:
        # particleFilter.visualize(myRobot, '10000', None, None)
        sensed_distance1=myRobot.sense()
        myRobot.rotate(omega,turn1)
        sensed_distance2 = myRobot.sense()
        """ 1. simulate the robot motion for each particle """
        particles = particleFilter.simulateRotate(turn1)
        """ 2. weight different particles, based on sensor data """
        weights = particleFilter.weightParticles(sensed_distance2)
        i =  np.argmax(weights)
        print("weights", np.argmax(weights), "p", id(particles[i]), "w", weights[i])
        """ 4. estimate position based on particles """
        estimated_position = particleFilter.estimatePosition()
        condition= particleFilter.find_dist()
        """ 3. resample, with more higher weighted particles """
        particles_sample = particleFilter.resampleParticles()
        # visualize
        particleFilter.visualize(myRobot, t, particles, estimated_position)
        t += 1

        if sensed_distance2>move_condition:
            myRobot.go(sp, dist)
            sensed_distance3 = myRobot.sense()           
            """ 1. simulate the robot motion for each particle """
            particles = particleFilter.simulateGo(dist)
            # if labels.count(most_frequent(labels))>0.8*len(labels):
            #     condition=False
            """ 2. weight different particles, based on sensor data """
            sensed_distances = myRobot.sense()
            weights = particleFilter.weightParticles(sensed_distance3)
            """ 4. estimate position based on particles """
            estimated_position = particleFilter.estimatePosition()
            condition= particleFilter.find_dist()
            """ 3. resample, with more higher weighted particles """
            particles_sample = particleFilter.resampleParticles()
            # visualize
            particleFilter.visualize(myRobot, t, particles, estimated_position)
            t += 1
        else:
            myRobot.rotate(omega, turn2)
            particles = particleFilter.simulateRotate(turn2)            
            if sensed_distance1>move_condition:
                myRobot.go(sp, dist)
                sensed_distance4 = myRobot.sense()
                """ 1. simulate the robot motion for each particle """
                particles = particleFilter.simulateGo(dist)  
                """ 2. weight different particles, based on sensor data """
                sensed_distances = myRobot.sense()
                weights = particleFilter.weightParticles(sensed_distance4)
                """ 4. estimate position based on particles """
                estimated_position = particleFilter.estimatePosition()
                condition= particleFilter.find_dist()
                """ 3. resample, with more higher weighted particles """
                particles_sample = particleFilter.resampleParticles()
                # visualize
                particleFilter.visualize(myRobot, t, particles, estimated_position)
                t += 1
            else:
                myRobot.rotate(omega, turn2)
                sensed_distance5 = myRobot.sense()
                """ 1. simulate the robot motion for each particle """
                particles = particleFilter.simulateRotate(turn2)
                """ 2. weight different particles, based on sensor data """
                weights = particleFilter.weightParticles(sensed_distance5)
                """ 4. estimate position based on particles """
                estimated_position = particleFilter.estimatePosition()
                condition= particleFilter.find_dist()
                """ 3. resample, with more higher weighted particles """
                particles_sample = particleFilter.resampleParticles()
                # visualize
                particleFilter.visualize(myRobot, t, particles, estimated_position)
                t += 1                


                myRobot.go(sp, dist)
                sensed_distance6 = myRobot.sense()
                """ 1. simulate the robot motion for each particle """
                particles = particleFilter.simulateGo(dist)
                """ 2. weight different particles, based on sensor data """
                sensed_distances = myRobot.sense()
                weights = particleFilter.weightParticles(sensed_distance6)
                """ 4. estimate position based on particles """
                estimated_position = particleFilter.estimatePosition()
                condition= particleFilter.find_dist()
                """ 3. resample, with more higher weighted particles """
                particles_sample = particleFilter.resampleParticles()
                # visualize
                particleFilter.visualize(myRobot, t, particles, estimated_position)        
                t+=1
    myRobot.prettyPrint()   
    # getting timing
    time_end = time.time()
    running_time = time_end - time_start
    print("Program run %.4f seconds for %s steps and %s particles"%(running_time, steps, particleFilter.n))
            
