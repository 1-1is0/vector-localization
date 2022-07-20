#!/usr/bin/env python
# -*- coding: utf-8 -*-
from shapely.geometry import LineString, Point
import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos
dots_list = [
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

arr = [np.array(i) for i in dots_list]
lines = []
dots_list = np.array(arr)
for i in dots_list:
    order = (0, 1, 3, 2, 0)
    pes = []
    for j in order:
        pes.append(Point(i[j, :]))
    lines.append(LineString(pes))


plt.clf()
for line in lines:
    plt.plot(*line.xy)
particle_distances = []
class PP:
    def __init__(self, x, y, orientation):
        self.x = x
        self.y = y
        self.orientation = orientation


particles = [PP(-40, -200,64), ]
for p in particles:
    start = Point(p.x, p.y)
    length = 1500
    end = Point(start.x + length * cos(p.orientation),
                start.y + length * sin(p.orientation)
    )
    print('start', start)
    print("end", end)
    line_p = LineString([start, end])
    plt.plot(*line_p.xy)
    dots_lines = []
    inter = []
    distances = []
    for line in lines:
        dots_lines.append(line)
        inter.append(line_p.intersection(line))
    for i in inter:
        if not i.is_empty:
            distances.append(start.distance(i))
    print(f"particle {p.x}, {p.y} {p.orientation}")
    print(distances)
    particle_distances.append(min(distances))
print(particle_distances)
plt.savefig("rects.png")
