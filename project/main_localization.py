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

    # choose steps
    #steps = 10
    # choose number of particles
    nParticles = 1000
    
    # set world
    #TODO سایز محیط و مختصات 4 گوشه دستی وارد شود
    length= 20 
    width= 20
    landmarks = [[-10.0, -10.0], [10.0, 10.0], [-10.0, 10.0], [10.0, -10.0], [0, 10]]

    world = World(length=20, width=20, landmarks=landmarks)

    # set particle filter
    #TODO set variances
    particleFilter = ParticleFilter(nParticles, forward_noise_dict[50], turn_noise_dict[90], sensor_noise_dict, world)
    # print("World size: %s"%particleFilter.WORLD.size)

    # set robot
    # موارد زیر به به ربات ماست و باید تغییر کند
    #اگر ربات از یک نقطه مشخص حرکت میند کد زیر درست و در غیر اینصورت غیر درست است
    #turning_pattern = [0, 1.57, 1.57, 1.0, -2.57, -1.57, 0.6, -0.4]
    #forward_pattern = [3, 2, 6, 4, 3, 2, 3, 2]
    #steps = len(turning_pattern)

    # start new node
    rospy.init_node("main_localization", anonymous=True)
    # initialize robot
    #TODO واریانس خطاهای مختلف ربات
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
    #TODO کدی نوشته شود که تا زمانی که بیش از 80 درصد ذرات در یک خوشه نیستند به کار خود ادامه دهد
    for t in range(steps):
        #rospy.loginfo("********* Step %s *********"%t)
        myRobot.prettyPrint()
                
        # move the robot in random direction
        #move_turn = random.uniform(-1.0, 1.0) * pi
        #move_forward = random.random() * 2
        #move_turn = turning_pattern[t]
        #move_forward = forward_pattern[t]
        sensed_distance1=myRobot.sense()
        myRobot.rotate(omega, turn1)
        sensed_distance2 = myRobot.sense()
        """ 1. simulate the robot motion for each particle """
        particles = particleFilter.simulateRotate(omega, turn1)
        """ 2. weight different particles, based on sensor data """
        weights = particleFilter.weightParticles(sensed_distance2)
        """ 3. resample, with more higher weighted particles """
        particles_sample = particleFilter.resampleParticles()
        """ 4. estimate position based on particles """
        estimated_position = particleFilter.estimatePosition()
        # visualize
        particleFilter.visualize(myRobot, t, particles, estimated_position)

        if sensed_distance2>50:
            myRobot.go(sp, dist)
            sensed_distance3 = myRobot.sense()           
            """ 1. simulate the robot motion for each particle """
            particles = particleFilter.simulateGo(sp, dist)
            """ 2. weight different particles, based on sensor data """
            sensed_distances = myRobot.sense()
            weights = particleFilter.weightParticles(sensed_distance3)
            """ 3. resample, with more higher weighted particles """
            particles_sample = particleFilter.resampleParticles()
            """ 4. estimate position based on particles """
            estimated_position = particleFilter.estimatePosition()
            # visualize
            particleFilter.visualize(myRobot, t+1, particles, estimated_position)
        else:
            myRobot.rotate(omega, turn2)
            particles = particleFilter.simulateRotate(omega, turn2)            
            if sensed_distance1>50:
                myRobot.go(sp, dist)
                sensed_distance4 = myRobot.sense()
                """ 1. simulate the robot motion for each particle """
                particles = particleFilter.simulateGo(sp, dist)
                """ 2. weight different particles, based on sensor data """
                sensed_distances = myRobot.sense()
                weights = particleFilter.weightParticles(sensed_distance4)
                """ 3. resample, with more higher weighted particles """
                particles_sample = particleFilter.resampleParticles()
                """ 4. estimate position based on particles """
                estimated_position = particleFilter.estimatePosition()
                # visualize
                particleFilter.visualize(myRobot, t+1, particles, estimated_position)
            else:
                myRobot.rotate(omega, turn2)
                sensed_distance5 = myRobot.sense()
                """ 1. simulate the robot motion for each particle """
                particles = particleFilter.simulateRotate(omega, turn2)
                """ 2. weight different particles, based on sensor data """
                weights = particleFilter.weightParticles(sensed_distance5)
                """ 3. resample, with more higher weighted particles """
                particles_sample = particleFilter.resampleParticles()
                """ 4. estimate position based on particles """
                estimated_position = particleFilter.estimatePosition()
                # visualize
                particleFilter.visualize(myRobot, t+1, particles, estimated_position)

                myRobot.go(sp, dist)
                sensed_distance6 = myRobot.sense()
                """ 1. simulate the robot motion for each particle """
                particles = particleFilter.simulateGo(sp, dist)
                """ 2. weight different particles, based on sensor data """
                sensed_distances = myRobot.sense()
                weights = particleFilter.weightParticles(sensed_distance6)
                """ 3. resample, with more higher weighted particles """
                particles_sample = particleFilter.resampleParticles()
                """ 4. estimate position based on particles """
                estimated_position = particleFilter.estimatePosition()
                # visualize
                particleFilter.visualize(myRobot, t+2, particles, estimated_position)

     
    # getting timing
    time_end = time.time()
    running_time = time_end - time_start
    print("Program run %.4f seconds for %s steps and %s particles"%(running_time, steps, particleFilter.n))
            
