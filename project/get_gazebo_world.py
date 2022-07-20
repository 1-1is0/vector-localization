#!/usr/bin/python python
# -*- coding: utf-8 -*-

import rospy
from math import isclose
from pyquaternion import Quaternion
from enum import Enum
from lxml import etree
import matplotlib.pyplot as plt
import numpy as np
from gazebo_msgs.srv import GetWorldProperties, GetModelState, GetModelProperties, GetLinkState

class GazeboService:
    # state -> post
    # prop -> child
    MODEL_STATE = "gazebo/get_model_state"
    MODEL_PROPERTIES = "gazebo/get_model_properties"
    WORLD_PROPERTIES = "gazebo/get_world_properties"
    LINK_STATE = "gazebo/get_link_state"

    @classmethod
    def get_model_pose(cls, model_name, relative="world"):
        func = cls.call_service_proxy(cls.MODEL_STATE, GetModelState)
        model_state = func(model_name=model_name, relative_entity_name=relative)
        return model_state.pose

    @classmethod
    def get_body_names(cls, model_name):
        """
        get models bodys list from model prop
        """
        func = cls.call_service_proxy(cls.MODEL_PROPERTIES, GetModelProperties)
        model_prop = func(model_name=model_name)
        return model_prop.body_names

    @classmethod
    def get_main_model_name(cls):
        """
        get main model name from world prop
        """
        func = cls.call_service_proxy(cls.WORLD_PROPERTIES, GetWorldProperties)
        result = func()
        models = result.model_names
        # which model to get
        main_model = models[1]
        print("main model", main_model)
        return main_model
    
    @classmethod
    def get_link_pose(cls, link_name, reference="world"):
        func = cls.call_service_proxy(cls.LINK_STATE, GetLinkState)
        result = func(link_name=link_name, reference_frame=reference)
        return result.link_state.pose

    @classmethod
    def get_link_collision(cls, model_name, link_name, world_file="sample5.world"):
        """
        read world file and exctract gome collision size from it
        """
        doc = etree.parse(world_file)
        col = doc.xpath(f'//model[@name="{model_name}"]/link[@name="{link_name}"]/collision/geometry/box/size')
        x, y, z = map(float, col[0].text.split(' '))
        return x, y, z
    
    @staticmethod
    def call_service_proxy(name, typ):
        return rospy.ServiceProxy(name, typ)
    
    
def calc_quat(ori1, ori2, pos):
    q1 = Quaternion(ori1.w, ori1.x, ori1.y, ori1.z)
    q2 = Quaternion(ori2.w, ori2.x, ori2.y, ori2.z)
    #  w= ori1.w + ori2.w
    #  x= ori1.x + ori2.x
    #  y= ori1.y + ori2.y
    #  z= ori1.z + ori2.z
    q = q1 * q2
    q_rotate = q2.rotate(pos)
    pos = list(pos)

    #  pos[0] = pos[0] + pos[0] * 0.1
    if not isclose(q_rotate[0], pos[0], rel_tol=0.05):
        pos = [pos[1], pos[0], pos[2]]
        if mode == 2:
            pos[1] = pos[1] + pos[1] * 0.15
    else:
        if mode == 2:
            pos[0] = pos[0] + pos[0] * 0.15
    return list(pos)

def get_main_model():
    world_file_dir = "/home/amir/uni/robotics/anki_description_ws/src/anki_description/world/sample1.world"
    model = GazeboService.get_main_model_name()
    model_pose = GazeboService.get_model_pose(model)
    body_names = GazeboService.get_body_names(model)
    real_position = [0, 0, 0]
    all_ans = []
    real_links = []
    for name in body_names:
        link_pose = GazeboService.get_link_pose(name)
        link_size = GazeboService.get_link_collision(model, name, world_file=world_file_dir)
        real_link_size = calc_quat(model_pose.orientation, link_pose.orientation, link_size)
        real_link_position = [0, 0, 0] # middle of the shape
        edges_positions = np.zeros((4, 3))  # 4 edge with 3 position
        for i, attr in enumerate(['x', 'y', 'z']):
            real_link_position[i] = getattr(link_pose.position, attr) - getattr(model_pose.position, attr)
            #  real_link_position[i] = getattr(link_pose.position, attr)

        print("##########")
        print(name)
        print("link", link_pose)
        print("real link ", real_link_position)


        print("link_size", link_size)
        print("real_link_size", real_link_size)

        if mode == 2:
            edges_positions = np.zeros((2, 3))  # 2 edge with 3 position
            amal = [(1,1), (-1, 1)]
            less = 1
            if real_link_size[0] < real_link_size[1]:  # x < y
                amal = [(1, 1), (1, -1)]
                less = 0
            real_link_size[less] /= 2  # x/2 or y/2
            print("after", real_link_size)
            for index, (sign_1, sign_2) in enumerate(amal):
                edges_positions[index, :] = np.array([
                    real_link_position[0] + sign_1 * real_link_size[0]/2,
                    real_link_position[1] + sign_2 * real_link_size[1]/2,
                    real_link_position[2],
                ])

        else:
            edges_positions = np.zeros((4, 3))  # 4 edge with 3 position
            for index, (sign_1, sign_2) in enumerate([(1,1), (1, -1), (-1, 1), (-1, -1)]):
                edges_positions[index, :] = np.array([
                    real_link_position[0] + sign_1 * real_link_size[0]/2,
                    real_link_position[1] + sign_2 * real_link_size[1]/2,
                    real_link_position[2],
                ])

    #
            print(edges_positions)
            edges_sum = np.sum(edges_positions, axis=1)
            temp = 0
            if edges_sum[0] < edges_sum[1]:
                #  swap them toghter
                edges_positions = np.array([edges_positions[1], edges_positions[0]])
    #
        #  input("enter enter")
#
        #  plt.plot(edges_positions[:, 0], edges_positions[:, 1])
        all_ans.append(edges_positions)
        real_links.append(real_link_position)
    return np.array(all_ans), np.array(real_links)

def draw_rect(points, pose):
    if mode == 2:
        order = (0, 1)
    else:
        order = (0, 1, 3, 2, 0)
    plt.clf()
    for i in range(len(points)):
        plt.plot(points[i, order, 0], points[i, order], 1)
    plt.scatter(pose[:, 0], pose[:, 1])
    #  print(pose)
    #  print(pose[:, 0])

    plt.ion()
    plt.show(block=False)
    plt.pause(10000)
    plt.close()
    plt.savefig("lines.png")

mode = 4
def get():
    
    ans, pose = get_main_model()
    print(ans)
    print(ans.shape)
    print(list(ans[:, :, :2]*1000))
    #  draw_rect(ans*1000, pose*1000)

get()

