import xml.etree.ElementTree as ET
# import matplotlib.patches as patches
import matplotlib.pyplot as plt
import math
import shapely
from shapely.geometry import LineString,Point, Polygon






# def init_map(address):

#     tree = ET.parse(address)
#     root = tree.getroot()


#     rects = []
#     centers = []

#     for links in root[0].iter('model'):
    
#         try:

#             for link in links: 
#                 if link.tag == 'pose':
#                     _global_map_pose = link.text.split(' ')
#                     if _global_map_pose[0] != '0':
#                         global_map_pose = _global_map_pose
#                         print(global_map_pose) 
                    

#             # for link in links: 
#                 if link.tag == 'link':

#                     geometry = None 
#                     pose = None
#                     for _pose in link.iter('pose'):
#                         pose = _pose.text.split(' ')
#                         break
#                     for collision in link.iter('collision'):
#                         geometry = collision[3][0][0].text.split(' ')


#                     p1 = [float(pose[0]) + (float(geometry[0]) * math.cos(float(pose[5])) / 2)
#                             + float(geometry[1]) * math.sin(float(pose[5])) / 2 ,
#                             float(pose[1]) + float(geometry[0]) * math.sin(float(pose[5])) / 2
#                                 - float(geometry[1]) * math.cos(float(pose[5])) / 2 ]

#                     p2 = [float(pose[0]) + float(geometry[0]) * math.cos(float(pose[5])) / 2
#                         -float(geometry[1])*math.sin(float(pose[5]))/2 ,
#                             float(pose[1]) + float(geometry[0])*math.sin(float(pose[5]))/2
#                                 +float(geometry[1])*math.cos(float(pose[5]))/2]

#                     p3 = [float(pose[0]) - float(geometry[0])*math.cos(float(pose[5]))/2
#                         +float(geometry[1])*math.sin(float(pose[5]))/2 ,
#                             float(pose[1]) - float(geometry[0])*math.sin(float(pose[5]))/2
#                                 -float(geometry[1])*math.cos(float(pose[5]))/2]

#                     p4 = [float(pose[0]) - float(geometry[0])*math.cos(float(pose[5]))/2
#                         -float(geometry[1])*math.sin(float(pose[5]))/2 ,
#                             float(pose[1]) - float(geometry[0])*math.sin(float(pose[5]))/2
#                                 +float(geometry[1])*math.cos(float(pose[5]))/2]


#                     rects.append([p1 ,p2 ,p3 ,p4])
#                     centers.append([pose[0],pose[1]])
                
#                     # plt.plot([p1[0],p2[0],p4[0],p3[0],p1[0]],[p1[1],p2[1],p4[1],p3[1],p1[1]])
                    
#         except Exception as e : pass
    
#     # print(global_map_pose)
    
#     return rects,global_map_pose,map_boundry(centers)

def init_map():

    # tree = ET.parse()
    # root = tree.getroot()


    rects = []
    centers = []

    wall = 16 #mm
    p1 = [0,0]
    p2 = [wall, 0]
    p3 = [2*wall+483+235,0]
    p4 = [p3[0]+wall,0]
    p5 = [wall, wall]
    p6 = [wall + 483, wall]
    p7 = [p6[0]+wall, wall]
    p8 = [p7[0]+235, wall]
    p9 = [p8[0]+wall,wall]
    p10 = [wall, wall + 245]
    p11 = [wall + 295, wall + 245]
    p12 = [wall + 295, wall + 245 - 90]
    p13 = [p12[0] + wall, p12[1]]
    p14 = [wall + 483, wall + 220]
    p15 = [p14[0] + wall, p14[1]]
    p16 = [p15[0] + 85, p15[1]]
    p17 = [p16[0], p16[1] + wall]
    p18 = [p14[0], p14[1] + wall]
    p19 = [p13[0], p13[1] + 200]
    p20 = [p12[0], p12[1] + 200]
    p21 = [p11[0], p11[1] + wall]
    p22 = [p10[0], p10[1] + wall]
    p23 = [wall, 2* wall + 245 + 237]
    p24 = [wall + 300, p23[1]]
    p25 = [p24[0], p24[1] + wall]
    p26 = [p23[0], p23[1] + wall]
    p27 = [wall, 3* wall + 245 + 237+225]
    p28 = [wall, p27[1] + wall]
    p29 = [0, p27[1] + wall]
    p30 = [p27[0] + 467, p27[1]]
    p31 = [p30[0] + wall, p30[1]]
    p32 = [p31[0] + 255, p31[1]]
    p33 = [p32[0], p32[1] + wall]
    p34 = [p33[0] + wall, p33[1]]
    p35 = [p31[0], p31[1] - 220]
    p36 = [p35[0] + 110 - wall - 45, p35[1]]
    p37 = [p36[0], p36[1] - wall]
    p38 = [p37[0] - 110, p37[1]]
    p39 = [p38[0], p38[1] + wall]
    p40 = [p39[0] + 45, p39[1]]
    p41 = [wall + 300, wall*2 + 245 + 237]
    p42 = [wall + 300, wall*3 + 245 + 237]
    p43 = [wall + 300 + 220, wall*3 + 245 + 237]
    p44 = [wall + 300 + 220, wall*2 + 245 + 237 - 5 - 40]
    p45 = [p44[0] + wall, p44[1]]
    p46 = [p44[0], p44[1] + 110]
    p47 = [p46[0] + wall, p46[1]]
    p48 = [wall + 300 + 220, wall*2 + 245 + 237] 
    rects = [[p1,p29,p2,p28], [p2,p5,p3,p8],[p3,p33,p4,p34],[p27,p28,p32,p33], [p10,p22,p11,p21], [p12,p20,p13,p19], [p6,p14,p7,p15],[p14,p18,p16,p17],[p23,p26,p24,p25], [p48,p41,p43,p42], [p44, p46, p45, p47]]#, [p40,p30,p35,p31], [p38, p39, p37, p36]
    centers=[]

    for i in range (len(rects)):
      x=(rects[i][0][0]+ rects[i][2][0])/2
      y=(rects[i][1][1]+rects[i][3][1])/2
      centers.append([x,y])
    global_map_pose=[0,0]
    return rects,global_map_pose,map_boundry(centers)
    
def find_intersection(p1,p2,p3,p4):

    line1 = LineString([tuple(p1), tuple(p2)])
    line2 = LineString([tuple(p3), tuple(p4)])

    int_pt = line1.intersection(line2)
    if int_pt:
        point_of_intersection = int_pt.x, int_pt.y
        return point_of_intersection
    else:
        return False


def convert_point_to_line(rects):
    lines = []
    for points in rects:
        lines.append([ points[0] , points[1]] )
        lines.append([ points[1] , points[3]] )
        lines.append([ points[3] , points[2]] )
        lines.append([ points[2] , points[0]] )
    return lines


def add_offset(rects,offset):
    new_rects = []
    for points in rects: 
        new_rects.append(
            [  
                [points[0][0] + offset[0] , points[0][1] + offset[1]  ] ,
                [points[1][0] + offset[0] , points[1][1] + offset[1]  ] ,
                [points[2][0] + offset[0] , points[2][1] + offset[1]  ] ,
                [points[3][0] + offset[0] , points[3][1] + offset[1]  ] 
            ]
        )
    return new_rects


def convert_to_poly(rects):
    polygons = []
    for points in rects:
        polygons.append(Polygon(
            [tuple(points[0]) ,
            tuple(points[1]),
            tuple(points[3]),
            tuple(points[2])
            ]))
    return polygons

def check_is_collition(point , rects):
    p = Point(tuple(point))
    for rect in rects:
        if rect.contains(p):
            return True
    return False


def map_boundry(centers):
    X = []
    Y = [] 

    for item in centers:
        X.append(float(item[0]))
        Y.append(float(item[1]))

    return min(X),max(X),min(Y),max(Y)
    



def out_of_range(particle,offset,map_boundry):
    if particle[0] - offset[0] > map_boundry[1] or particle[0] - offset[0] < map_boundry[0]:
        return True
    elif particle[1] - offset[1] > map_boundry[3] or particle[1] - offset[1] < map_boundry[2]:
        return True
    else:
        return False

def plot_map(rects):
    for rect in rects:
        rect = list(zip(*rect))
        plt.plot(rect[1], rect[0], c='black')






