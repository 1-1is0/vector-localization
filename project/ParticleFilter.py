import time
from math import *
import random
from turtle import distance
from util import *
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rospy

""" Particle Filter class, which includes the particles class """
class ParticleFilter:
    # global variable world
    WORLD = -1
    
    def __init__(self, numberOfParticles, forward_noise, turn_noise, sense_noise, world):
        # set world as global variable
        ParticleFilter.WORLD = world
        # number of particles
        self.n = numberOfParticles
        # initialize weights to 1/n
        self.weights = [1.0/self.n] * self.n
        # create n number of particles
        particles = []
        for i in range(numberOfParticles):
            x = self.Particle()
            x.setNoise(forward_noise, turn_noise)
            particles.append(x)
        self.particles = particles
        self.sense_noise = sense_noise

    def showParticles(self):
        print("Show all particles")
        for i in range(self.n):
            p = self.particles[i]
            print(f"Particle {i}:  position = ({p.x:.2f}, {p.y:.2f}) \torientation = {p.orientation:.2f} \tweight = {self.weight}")
    
    def simulateGo(self, speed, distance, turn):
        rospy.loginfo("Simulate translation for particles")
        particles_new = []
        for i in range(self.n):
            particles_new.append(self.particles[i].go(speed, distance, turn))
        self.particles = particles_new
    def simulateRotate(self, speed, turn):
        rospy.loginfo("Simulate rotation for particles")
        particles_new = []
        for i in range(self.n):
            particles_new.append(self.particles[i].rotate(turn))
        self.particles = particles_new        
        return particles_new
    def particle_distance(self,x0,y0,theta,x1,y1,x2,y2):
        # maybe use laser behavior 
        if tan(theta)==(y2-y1)/(x2-x1):
            return False, 0
        else:
            x=(tan(theta)*x0-y0-((y2-y1)/(x2-x1))*x1+y1)/(tan(theta)-((y2-y1)/(x2-x1)))
            y=tan(theta)*(x-x0)+y0
            if (x>x1 and x<x2)and(y>y1 and y<y2):
                p_distance=sqrt(pow((x-x0),2)+pow((y-y0),2))
                return True, p_distance
            return False, 0
    def allParticleDistance(self):
        particle_distances = []
        p_dist = []
        for i in range(self.n):
            particle_distances = []
            if i ==1 :
                print("in self.n for")
            for j in range(int(len(dots_list)/2)):
                condition, p_distance = self.particle_distance(self.particles[i].x,self.particles[i].y,self.particles[i].orientation,dots_list[2*j][0],dots_list[2*j][1],dots_list[2*j+1][0],dots_list[2*j+1][1])
                if condition:
                    print("TRUE")
                    particle_distances.append(p_distance)

            #  print("len particle list", len(particle_distances))
            if len(particle_distances):
                p_dist.append(min(particle_distances))
        print("pdist", len(p_dist))
        return p_dist
    # TODO تعریف فاصله هر ذره تا دیوار رو به روییش به عنوان p_distance
    def p_distance(self,x,y,orientation):
        return np.random.randint(1, 10)

    def weightParticles(self, sensor_distance):
        rospy.loginfo("Weight the particles")
        # weight the particles based on the sensor data
        weights_new = []
        #TODO self.particles[i] may not be correct below

        if (sensor_distance < 100):
            key = 50
        elif (100 < sensor_distance < 200):
            key = 100
        elif (200 < sensor_distance < 300):
            key = 200
        elif (300 < sensor_distance < 370):
            key = 300
        else:
            key = 370
        mean, std =  self.sense_noise[key]

        distance_list = self.allParticleDistance()
        print(len(distance_list))
        for i in range(self.n):
            try:
                distance = distance_list[i]
            except:
                print("EXCEPTION", i)
            # distance = self.p_distance[self.particles[i]]

            # particle + mean(noise)
            weights_new.append(self.particles[i].getGaussianP(sensor_distance, std, distance))

        # normalize weight weights
        weights_new = normalize(weights_new)
        self.weights = weights_new
        return weights_new

    def resampleParticles(self):
        rospy.loginfo("Resample particles")
        resampled_particles = []
        # resampling algorithm
        index = int(random.random() * self.n)
        beta = 0.0
        max_weights = max(self.weights)
        for i in range(self.n):
            # get random index
            beta += random.random() * 2.0 * max_weights
            while beta > self.weights[index]:
                beta -= self.weights[index]
                index = (index+1) % self.n
            # append random particles
            resampled_particles.append( self.particles[index] )
        self.particles = resampled_particles
        
        return resampled_particles
    
    def estimatePosition(self):
        rospy.loginfo("Estimate new position")
        #TODO در قسمت زیر خوشه بندی کرده و از روی خوشه متراکم تر موقعیت تخمینی ربات را به دست میاوریم
        all_x = []
        all_y = []
        all_orientation = []
        for idx, part in enumerate(self.particles):#TODO not self.particles just the particles in the biggest cluster
            #print("X: %.3f \t Y: %.3f \t O: %.3f \t Weights: %.3f"%(part.x, part.y, part.orientation, self.weights[idx]))
            all_x.append( part.x * self.weights[idx] )
            all_y.append( part.y * self.weights[idx] )
            all_orientation.append( part.orientation )
        x_estimate = np.sum(all_x)
        y_estimate = np.sum(all_y)
        orientation_estimate = mean_circular_quantities(all_orientation, self.weights)
        print("Estimated position: (%.3f, %.3f)\nEstimated rotation: %.3f"%(x_estimate, y_estimate, orientation_estimate))
        return (x_estimate, y_estimate, orientation_estimate)

    def visualize(self, step, old_particles, estimated_position):
        rospy.loginfo("Visualize particle filtering")
        markersize = 100
        # arrow
        scale = 0.2
        width = 0.1
        # plotting
        plt.figure("Robot", figsize=(20., 20.))
        plt.title('Particle filter, step ' + str(step))
        # draw coordinate grid for plotting
        grid = [(-ParticleFilter.WORLD.length/2)-1, (ParticleFilter.WORLD.length/2)+1, (-ParticleFilter.WORLD.width/2)-1, (ParticleFilter.WORLD.width/2)+1]
        plt.axis(grid)
        plt.grid(b=True, which='major', color='0.75', linestyle='--')
        plt.xticks([i for i in range(-int(ParticleFilter.WORLD.length/2)-1, int(ParticleFilter.WORLD.length/2)+1, 1)])
        plt.yticks([i for i in range(-int(ParticleFilter.WORLD.width/2)-1, int(ParticleFilter.WORLD.width/2)+1, 1)])
        # draw particles
        for old_part in old_particles:
            plt.scatter(old_part.x, old_part.y, s=markersize, color="yellow", alpha=0.2)
            # orientation
            arrow = plt.Arrow(old_part.x, old_part.y, scale*cos(old_part.orientation), scale*sin(old_part.orientation),width=width, alpha=0.2, facecolor='yellow', edgecolor='black')
            plt.gca().add_patch(arrow)
        # draw resampled particles
        for part in self.particles:
            # particle
            plt.scatter(part.x, part.y, s=markersize, color="green", alpha=0.2)
            # particle's orientation
            arrow = plt.Arrow(part.x, part.y, scale*cos(part.orientation), scale*sin(part.orientation), width=width,alpha=0.2, facecolor='green', edgecolor='black')
            plt.gca().add_patch(arrow)
        # draw landmarks
        for lm in ParticleFilter.WORLD.landmarks:
            plt.scatter(lm[0], lm[1], s=markersize*3, color="red")
        
        # draw true robot position & orientation
        #plt.scatter(robot.trueX, robot.trueY, s=markersize*6, color="black", marker="s", label="Truth")
        #arrow = plt.Arrow(robot.trueX, robot.trueY, scale*cos(robot.trueOrientation), scale*sin(robot.trueOrientation),width=width, facecolor='black', edgecolor='black')
        #plt.gca().add_patch(arrow)
        # draw robot position & orientation according to movement
        #plt.scatter(robot.x, robot.y, s=markersize*6, color="blue", marker="v", label="Estimate")
        #arrow = plt.Arrow(robot.x, robot.y, scale*cos(robot.orientation), scale*sin(robot.orientation),width=width, facecolor='blue', edgecolor='blue')
        #plt.gca().add_patch(arrow)
        # draw estimated robot position & orientation
        #TODO estmated_position or estimatePosition????????????
        # x, y, orientation = estimated_position
        # plt.scatter(x, y, s=markersize*6, color="cyan", marker="v", label="Estimate")
        # arrow = plt.Arrow(x, y, scale*cos(orientation), scale*sin(orientation),width=width, facecolor='cyan', edgecolor='cyan')
        # plt.gca().add_patch(arrow)
        # save figure
        print("Save figure...")
        plt.savefig("data/figure_" + str(step) + ".png")
        plt.savefig("data/static.png")
        plt.close()

    """ Particle class for each particle in the particle filter """
    class Particle:
        def __init__(self):
            self.x = random.uniform(-ParticleFilter.WORLD.length/2, ParticleFilter.WORLD.length/2)
            self.y = random.uniform(-ParticleFilter.WORLD.width/2, ParticleFilter.WORLD.width/2)
            self.orientation = (random.choice([0,1,2,3])/2) * pi
            #TODO خطاهای ترنزلیشن، روتیشن در اینجا
            # noise for the moving process
            self.forward_noise = None
            self.turn_noise = None

        def prettyPrint(self):
            print("Particle position: (%.2f, %.2f) \t Orientation: %.2f"%(self.x, self.y, self.orientation))
       

        def setNoise(self, new_forward_noise, new_turn_noise):
            # forward movement
            self.forward_noise = new_forward_noise
            # for turning
            self.turn_noise = new_turn_noise

        # get probability of x, given mu and sigma
        def getGaussianP(self, mu, sigma, x):
            # see https://en.wikipedia.org/wiki/Gaussian_function
            variance = float(sigma)**2
            denominator = sqrt(2*pi*variance)
            numerator = exp( - (float(x) - float(mu))**2 / (2*variance))
            return numerator / denominator

        def setPosition(self, new_x, new_y, new_orientation):
            # TODO check values
            # assign values
            self.x = float(new_x)
            self.y = float(new_y)
            self.orientation = float(new_orientation)

        #def sense(self):
         #   allDistances = []
          #  # for each landmark, get the distance and add some noise
           # for lm in ParticleFilter.WORLD.landmarks:
            #    distance = getDistance( (self.x, self.y), lm )
             #   distance += random.gauss(0.0, self.sense_noise)
              #  allDistances.append(distance)
            #return allDistances
        #TODO حرکت پارتیکل ها که باید ویرایش شود
        #TODO بررسی کنیم که اگر ایکس یا ایگرگ پارتیکل خارج از محدوده محیط بود لیمیت شود
        def go(self, speed, distance, turn):
            print(f"Go forward Particle: speed={speed}, distance={distance}")
            

            orientation = self.orientation + float(turn) + np.random.normal(self.turn_noise[0], self.turn_noise[1])
            # has to be within range [-pi, pi]
            orientation = (orientation + pi) % (2 * pi) - pi
            # moving forward plus noise
            distance = float(distance) + np.random.normal(self.forward_noise[0], self.forward_noise[1])
            x = self.x + (np.cos(orientation) * distance)
            y = self.y + (np.sin(orientation) * distance)
            # check boundaries
            x = limit(x, -ParticleFilter.WORLD.size/2, ParticleFilter.WORLD.size/2)
            y = limit(y, -ParticleFilter.WORLD.size/2, ParticleFilter.WORLD.size/2)

            # create new particle
            newParticle = ParticleFilter.Particle()
            newParticle.setPosition(x, y, orientation)
            newParticle.setNoise(self.forward_noise, self.turn_noise)
            return newParticle

        #TODO مدل خطای روتیشن در زیر قید شود
        def rotate(self, turn):
            print(f"Rotate Particle: turn={turn:0.3f}")

            orientation = self.orientation + float(turn) + np.random.normal(self.turn_noise[0], self.turn_noise[1])
            # has to be within range [-pi, pi]
            orientation = (orientation + pi) % (2 * pi) - pi
            newParticle = ParticleFilter.Particle()
            newParticle.setPosition(self.x, self.y, orientation)
            newParticle.setNoise(self.forward_noise, self.turn_noise)
            return newParticle

