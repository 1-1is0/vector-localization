#!/usr/bin/env python3

# Credit to
# https://github.com/betab0t/vector_ros_driver/blob/develop/src/vector_ros_driver/camera.py

import rospy
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image


class Camera:
    def __init__(self, async_robot, publish_rate=10, image_frame_id="camera_link"):
        self.async_robot = async_robot
        self.async_robot.camera.init_camera_feed()
        self.rate = rospy.Rate(publish_rate)
        self.image_frame_id = image_frame_id
        self.image_publisher = rospy.Publisher("/camera", Image, queue_size=1)
        self.publish_camera_feed()

    def publish_camera_feed(self):
        bridge = cv_bridge.CvBridge()

        while (
            not rospy.is_shutdown()
            and self.async_robot.camera.image_streaming_enabled()
        ):
            image = bridge.cv2_to_imgmsg(
                np.asarray(self.async_robot.camera.latest_image.raw_image),
                encoding="rgb8",
            )  # convert PIL.Image to ROS Image
            image.header.stamp = rospy.Time.now()
            image.header.frame_id = self.image_frame_id
            self.image_publisher.publish(image)

            self.rate.sleep()
