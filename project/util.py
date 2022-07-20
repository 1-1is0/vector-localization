""" some helper functions for the mobile robotics project """
from math import *
from geometry_msgs.msg import Twist
import numpy as np

# get probability of x, given mu and sigma
def getGaussianP(mu, sigma, x):
    variance = float(sigma)**2
    denominator = sqrt(2*pi*variance)
    numerator = exp( - (float(x) - float(mu))**2 / (2*variance))
    return numerator / denominator

# limit a number
def limit(x, min_x, max_x):
    return max( min(max_x, x), min_x)

# mean of circular quantities according to https://en.wikipedia.org/wiki/Mean_of_circular_quantities
def mean_circular_quantities(alpha_arr, weights):
    sum_sin = 0
    sum_cos = 0
    for i in range(len(alpha_arr)):
        alpha = alpha_arr[i]
        weight = weights[i]
        sum_sin += weight * sin(alpha)
        sum_cos += weight * cos(alpha)
    return atan2(sum_sin, sum_cos)

# normalize array (e.g. weights)
def normalize(arr):
    sum_arr = np.sum(arr)
    normalized_arr = [i/sum_arr for i in arr]
    return normalized_arr
    
# class for the velocity message
class VelMsg:
    def __init__(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.msg = vel_msg

    def setForward(self, speed):
        self.msg.linear.x = speed
    def setTurn(self, omega):
        self.msg.angular.z = omega
# x1, y1, x2, y2 (we wrote the code in someway that always 
# the first point coordination must be smaller than the second one means that for example:
# in (2,5) and (2,3) we must first wite (2,3) in the list and then go for the second one)
#  dots_list = [(1,2),(5,5), (6,7), (10,15), (1,2), (1, 15)]
dots_list = [
    [[-614.14 , -496.493],       [ 535.86 , -496.493]],  
    [[ 535.86 , -546.493],       [ 535.86 ,  603.507]],  
    [[-564.14 , -546.493],       [-564.14 ,  603.507]],  
    [[-614.14 ,  603.507],       [ 535.86 ,  603.507]],  
    [[-184.94435,   26.189  ],   [ 505.49035,   26.189]],
    [[-567.33665, -236.09   ],   [-128.88535, -236.09 ]],
    [[-563.5703,  279.4   ],     [-182.8697,  279.4  ]], 
    [[ 172.396, -259.688],       [ 375.992, -259.688]],  
    [[ 15.812 ,  16.4238],       [ 15.812 , 242.9462]],  
    [[206.887 , 316.7498],       [206.887 , 543.2722]],  
    [[-342.282 , -245.8552],     [-342.282 ,  -19.3328]],
    [[336.962 ,  16.2868],       [336.962 , 242.8092]],  
    [[  55.292 , -200.5682],     [  55.292 ,   25.9542]],
    [[ 200.753 , -486.4452],     [ 200.753 , -259.9228]],
]


rects_list = [
    [[ 460.86 , -471.493],[ 460.86 , -571.493],[-539.14 , -471.493],[-539.14 , -571.493]],
    [[ 560.86 ,  528.507],[ 560.86 , -471.493],[ 460.86 ,  528.507],[ 460.86 , -471.493]],
    [[-539.14 ,  528.507],[-539.14 , -471.493],[-639.14 ,  528.507],[-639.14 , -471.493]],
    [[ 460.86 ,  628.507],[ 460.86 ,  528.507],[-539.14 ,  628.507],[-539.14 ,  528.507]],
    [[ 460.462,   31.189],[ 460.462,   11.189],[-139.916,   31.189],[-139.916,   11.189]],
    [[-157.48 , -231.09 ],[-157.48 , -251.09 ],[-538.742, -231.09 ],[-538.742, -251.09 ]],
    [[-207.698,  284.4  ],[-207.698,  264.4  ],[-538.742,  284.4  ],[-538.742,  264.4  ]],
    [[ 362.714, -254.688],[ 362.714, -274.688],[ 185.674, -254.688],[ 185.674, -274.688]],
    [[ 20.812, 228.173],[ 20.812,  31.197],[  0.812, 228.173],[  0.812,  31.197]],
    [[211.887, 528.499],[211.887, 331.523],[191.887, 528.499],[191.887, 331.523]],
    [[-337.282,  -34.106],[-337.282, -231.082],[-357.282,  -34.106],[-357.282, -231.082]],
    [[341.962, 228.036],[341.962,  31.06 ],[321.962, 228.036],[321.962,  31.06 ]],
    [[  60.292,   11.181],[  60.292, -185.795],[  40.292,   11.181],[  40.292, -185.795]],
    [[ 205.753, -274.696],[ 205.753, -471.672],[ 185.753, -274.696],[ 185.753, -471.672]]]

rects_list = [np.array(i) for i in rects_list]
lines = []
rects_list = np.array(rects_list)
