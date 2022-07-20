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
from shapely.geometry import LineString, Point
from sklearn.cluster import MeanShift, estimate_bandwidth

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
    
    def simulateGo(self,distance):
        rospy.loginfo(f"Simulate translation for particles {distance}")
        particles_new = []
        for i in range(self.n):
            particles_new.append(self.particles[i].go(distance))
        self.particles = particles_new
    def simulateRotate(self,turn):

        rospy.loginfo(f"Simulate rotation for particles {turn}")
        particles_new = []
        for i in range(self.n):
            particles_new.append(self.particles[i].rotate(turn))
        self.particles = particles_new        
        return particles_new

    def allParticleDistance(self):
        particle_distances = []
    
        for p in self.particles:
            plt.clf()
            start = Point(p.x, p.y)
            length = 1500
            end = Point(start.x + length * cos(p.orientation),
                        start.y + length * sin(p.orientation)
            )
            line_p = LineString([start, end])

            rects_lines = []
            inter = []
            distances = []
            for i in rects_list:
                order = (0, 1, 3, 2, 0)  # order to make rect
                pes = []
                for j in order:
                    pes.append(Point(i[j, :]))
                rect = LineString(pes)
                rects_lines.append(rect)
                inter.append(line_p.intersection(rect))

                # plt.plot(*rect.xy)
            # plt.plot(*line_p.xy)
            # plt.savefig(f"data/p/particle{id(p)}.png")

            # for line xy
            #  dots_lines = []
            #  for dot in dots_list:
                #  line = LineString([dot[0], dot[1]])
                #  dots_lines.append(line)
                #  inter.append(line_p.intersection(line))
            for i in inter:
                if not i.is_empty:
                    distances.append(start.distance(i))

            #  print(f" particle {p.x}, {p.y} {p.orientation}")
            #  print(distances)
            # print(distances)
            # input("enter to con")
            particle_distances.append(min(distances))

        # for i in range(self.n):
        #     p_dist = []
        #     print(f"{self.particles[i].x}, {self.particles[i].y} {self.particles[i].orientation}")
        #     if self.particles[i].orientation<0:
        #         self.particles[i].orientation=-self.particles[i].orientation
        #         self.particles[i].orientation= self.particles[i].orientation%(2*pi)
        #         self.particles[i].orientation=-self.particles[i].orientation
        #         self.particles[i].orientation=-self.particles[i].orientation+2*pi
        #     else:
        #         self.particles[i].orientation = self.particles[i].orientation-(self.particles[i].orientation//2*pi)*(2*pi)
        #         self.particles[i].orientation= self.particles[i].orientation%(2*pi)
        #     #  print(self.particles[i].orientation)
        #     for j in range(int(len(dots_list)/2)):
        #     #  print("forrrrrr")
        #     #  print(self.particles[i].orientation)
        #     #  print( self.particles[i].x)
        #     #  print( self.particles[i].y)
        #     #  print(dots_list[2*j][1])
        #     #  print(dots_list[2*j+1][1])
        #         if self.particles[i].orientation>0 and self.particles[i].orientation<pi/2 and dots_list[2*j][0]==dots_list[2*j+1][0] and dots_list[2*j][0]>self.particles[i].x:
        #             print("1")
        #             if dots_list[2*j][1]<tan(self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x)+self.particles[i].y and tan(self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x)+self.particles[i].y<dots_list[2*j+1][1]:
        #                 print("1inside")
        #                 dist=sqrt(pow((dots_list[2*j][0]-self.particles[i].x),2)+pow(tan(self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x),2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #         if self.particles[i].orientation>0 and self.particles[i].orientation<pi/2 and dots_list[2*j][1]==dots_list[2*j+1][1] and dots_list[2*j][1]>self.particles[i].y:
        #             print("2")
        #             if dots_list[2*j][0]<tan(pi/2-self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y)+self.particles[i].x and tan(pi/2-self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y)+self.particles[i].x<dots_list[2*j+1][0]:
        #                 print("2inside")
        #                 dist=sqrt(pow(tan(pi/2-self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y),2)+pow(dots_list[2*j][1]-self.particles[i].y,2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #         if self.particles[i].orientation>pi/2 and self.particles[i].orientation<pi and dots_list[2*j][0]==dots_list[2*j+1][0] and dots_list[2*j][0]<self.particles[i].x:
        #             print("3")
        #             if dots_list[2*j][1]<tan(pi-self.particles[i].orientation)*(-dots_list[2*j][0]+self.particles[i].x)+self.particles[i].y and tan(pi-self.particles[i].orientation)*(-dots_list[2*j][0]+self.particles[i].x)+self.particles[i].y<dots_list[2*j+1][1]:
        #                 print("3inside")
        #                 dist=sqrt(pow((dots_list[2*j][0]-self.particles[i].x),2)+pow(tan(pi-self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x),2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #         if self.particles[i].orientation>pi/2 and self.particles[i].orientation<pi and dots_list[2*j][1]==dots_list[2*j+1][1] and dots_list[2*j][1]>self.particles[i].y:
        #             print("4")
        #             if dots_list[2*j][0]<-tan(-pi/2+self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y)+self.particles[i].x and -tan(-pi/2+self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y)+self.particles[i].x<dots_list[2*j+1][0]:
        #                 print("4inside")
        #                 dist=sqrt(pow(tan(-pi/2+self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y),2)+pow(dots_list[2*j][1]-self.particles[i].y,2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #         if self.particles[i].orientation>pi and self.particles[i].orientation<3*pi/2 and dots_list[2*j][0]==dots_list[2*j+1][0] and dots_list[2*j][0]<self.particles[i].x:
        #             print("5")
        #             if dots_list[2*j][1]<tan(-pi+self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x)+self.particles[i].y and tan(-pi+self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x)+self.particles[i].y<dots_list[2*j+1][1]:
        #                 print("5inside")
        #                 dist=sqrt(pow((dots_list[2*j][0]-self.particles[i].x),2)+pow(tan(-pi+self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x),2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #         if self.particles[i].orientation>pi and self.particles[i].orientation<3*pi/2 and dots_list[2*j][1]==dots_list[2*j+1][1] and dots_list[2*j][1]<self.particles[i].y:
        #             print("6")
        #             if dots_list[2*j][0]<tan(3*pi/2-self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y)+self.particles[i].x and tan(3*pi/2-self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y)+self.particles[i].x<dots_list[2*j+1][0]:
        #                 print("6inside")
        #                 dist=sqrt(pow(tan(3*pi/2-self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y),2)+pow(dots_list[2*j][1]-self.particles[i].y,2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #         if self.particles[i].orientation>3*pi/2 and self.particles[i].orientation<2*pi and dots_list[2*j][0]==dots_list[2*j+1][0] and dots_list[2*j][0]>self.particles[i].x:
        #             print("7")
        #             if dots_list[2*j][1]<tan(2*pi-self.particles[i].orientation)*(-dots_list[2*j][0]+self.particles[i].x)+self.particles[i].y and tan(2*pi-self.particles[i].orientation)*(-dots_list[2*j][0]+self.particles[i].x)+self.particles[i].y<dots_list[2*j+1][1]:
        #                 print("7inside")
        #                 dist=sqrt(pow((dots_list[2*j][0]-self.particles[i].x),2)+pow(tan(2*pi-self.particles[i].orientation)*(dots_list[2*j][0]-self.particles[i].x),2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #         if self.particles[i].orientation>3*pi/2 and self.particles[i].orientation<2*pi and dots_list[2*j][1]==dots_list[2*j+1][1] and dots_list[2*j][1]<self.particles[i].y:
        #             print("8")
        #             if dots_list[2*j][0]<tan(-3*pi/2+self.particles[i].orientation)*(-dots_list[2*j][1]+self.particles[i].y)+self.particles[i].x and tan(-3*pi/2+self.particles[i].orientation)*(-dots_list[2*j][1]+self.particles[i].y)+self.particles[i].x<dots_list[2*j+1][0]:
        #                 print("8inside")
        #                 dist=sqrt(pow(tan(-3*pi/2+self.particles[i].orientation)*(dots_list[2*j][1]-self.particles[i].y),2)+pow(dots_list[2*j][1]-self.particles[i].y,2))
        #                 if dist>400:
        #                     p_dist.append(400)
        #                 else:
        #                     p_dist.append(dist)
        #     particle_distances.append(min(p_dist))
        # print(len(particle_distances))

        return particle_distances

    def weightParticles(self, sensor_distance):
        rospy.loginfo("Weight the particles")
        # weight the particles based on the sensor data
        weights_new = []
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
            distance = distance_list[i]
            #weights_new.append(self.particles[i].getGaussianP(sensor_distance, std, distance))
            weights_new.append(getGaussianP(sensor_distance, std, distance))
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
            resampled_particles.append(self.particles[index])
        self.particles = resampled_particles        
        return resampled_particles
    
    
    # def resampleParticles(self):
    #     """ Performs the systemic resampling algorithm used by particle filters.
    #     This algorithm separates the sample space into N divisions. A single random
    #     offset is used to to choose where to sample from for all divisions. This
    #     guarantees that every sample is exactly 1/N apart.
    #     Parameters
    #     ----------
    #     weights : list-like of float
    #         list of weights as floats
    #     Returns
    #     -------
    #     indexes : ndarray of ints
    #         array of indexes into the weights defining the resample. i.e. the
    #         index of the zeroth resample is indexes[0], etc.
    #     """
    #     N = len(self.weights)

    #     # make N subdivisions, and choose positions with a consistent random offset
    #     positions = (np.random.random() + np.arange(N)) / N

    #     indexes = np.zeros(N, 'i')
    #     cumulative_sum = np.cumsum(self.weights)
    #     i, j = 0, 0
    #     while i < N:
    #         if positions[i] < cumulative_sum[j]:
    #             indexes[i] = j
    #             i += 1
    #         else:
    #             j += 1
    #     ps = []
    #     print("indexes", indexes)
    #     for i in indexes:
    #         ps.append(self.particles[i])
    #     self.particles = ps
    #     return ps

    def most_frequent(self,List):
        return max(set(List), key = List.count)
    
    def estimatePosition(self):
        # rospy.loginfo("Estimate new position")           
        all_x = []
        all_y = []
        all_orientation = []
        for idx, part in enumerate(self.particles):
            #print("X: %.3f \t Y: %.3f \t O: %.3f \t Weights: %.3f"%(part.x, part.y, part.orientation, self.weights[idx]))
            all_x.append( part.x * self.weights[idx] )
            all_y.append( part.y * self.weights[idx] )
            all_orientation.append( part.orientation )
        x_estimate = np.sum(all_x)
        y_estimate = np.sum(all_y)
        orientation_estimate = mean_circular_quantities(all_orientation, self.weights)
        # print("Estimated position: (%.3f, %.3f)\n"%(x_estimate, y_estimate))
        return (x_estimate, y_estimate,orientation_estimate)

    def find_dist(self):
        num = 0
        for i in range(self.n):
            x_estimate,y_estimate, orientation_estimate = self.estimatePosition()
            di=dist([x_estimate,y_estimate],[self.particles[i].x,self.particles[i].y])
            if di<50:
                num=num+1
        if num>0.8*self.n:
            return False
        else:
            return True

    def visualize(self, robot, step, old_particles, estimated_position):
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
        #  for old_part in old_particles:
            #  plt.scatter(old_part.x, old_part.y, s=markersize, color="yellow", alpha=0.2)
            #  orientation
            #  arrow = plt.Arrow(old_part.x, old_part.y, scale*cos(old_part.orientation), scale*sin(old_part.orientation),width=width, alpha=0.2, facecolor='yellow', edgecolor='black')
            #  plt.gca().add_patch(arrow)
        #  draw resampled particles
        print("in vis prin len particle", len(self.particles))
        for part in self.particles:
            # particle
            plt.scatter(part.x, part.y, s=markersize, color="green", alpha=0.2)
            # particle's orientation
            arrow = plt.Arrow(part.x, part.y, scale*cos(part.orientation), scale*sin(part.orientation), width=width,alpha=0.2, facecolor='green', edgecolor='black')
            plt.gca().add_patch(arrow)
        # draw landmarks
        for lm in ParticleFilter.WORLD.landmarks:
            plt.scatter(lm[0], lm[1], s=markersize*3, color="red")
        x, y, orientation = self.estimatePosition()
        plt.scatter(x, y, s=markersize*6, color="cyan", marker="v", label="Estimate")
        arrow = plt.Arrow(x, y, scale*cos(orientation), scale*sin(orientation),width=width, facecolor='cyan', edgecolor='cyan')
        plt.gca().add_patch(arrow)
        #save figure
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

        def setPosition(self, new_x, new_y, new_orientation):
            # TODO check values
            # assign values
            self.x = float(new_x)
            self.y = float(new_y)
            self.orientation = float(new_orientation)

        def go(self,distance):
            # print(f"Go forward Particle: distance={distance}")
            # moving forward plus noise
            # selfforward_noise[0] must be difference between the the true distance value and the one that robot realy moves                       
            # distance = distance + np.random.normal(0, self.forward_noise[1])
            distance = distance + np.random.normal(0, 2)
            x = self.x + (np.cos(self.orientation) * distance)
            y = self.y + (np.sin(self.orientation) * distance)
            
            # check boundaries
            x = limit(x, -ParticleFilter.WORLD.length/2, ParticleFilter.WORLD.length/2)
            y = limit(y, -ParticleFilter.WORLD.width/2, ParticleFilter.WORLD.width/2)

            # create new particle
            newParticle = ParticleFilter.Particle()
            newParticle.setPosition(x, y, self.orientation)
            newParticle.setNoise(self.forward_noise, self.turn_noise)
            return newParticle

        def rotate(self, turn):
            # print(f"Rotate Particle: turn={turn:0.3f}")
            # self.turn_noise[0] must be difference between the the true turn value and the one that robot realy turns
            # that in our cas in think is +7 degrees
            # orientation = self.orientation + float(turn) + np.random.normal(0, self.turn_noise[1])
            orientation = self.orientation + float(turn) + np.random.normal(0, 1)
            newParticle = ParticleFilter.Particle()
            newParticle.setPosition(self.x, self.y, orientation)
            newParticle.setNoise(self.forward_noise, self.turn_noise)
            return newParticle

