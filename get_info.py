#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from optitrack_ros.msg import or_pose_estimator_state

from cv_bridge import CvBridge, CvBridgeError
import cv2

bridge = CvBridge()
obj_pos = []

class Subscriber_tless:
    def __init__(self, name):
        rospy.init_node(name)
        self.sub1 = rospy.Subscriber("/optitrack/bodies/wand_gepetto", or_pose_estimator_state, lambda data: self.callback_pos_wand(data))

    def callback_pos_wand(self, data):
        rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub1.unregister()
            obj_pos = data.pos
            #store data in yaml in new dir

class Subscriber_tiago:
    #either on last directory : if empty. To override date from an existing one : precise the dir name
    def __init__(self, name):
        rospy.init_node(name)
        self.sub1 = rospy.Subscriber("/optitrack/bodies/tiago_base", or_pose_estimator_state, lambda data: self.callback_pos_tiago(data))
        self.sub2 = rospy.Subscriber("/xtion/rgb/images_raw", Image, lambda data: self.callback_image(data))

    def callback_pos_tiago(self, data):
        rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub1.unregister()
            #store data in yaml

    def callback_image(self, data):
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            cv2.imwrite('image_rgb.png', cv2_img)
            self.sub2.unregister()


def listener():
    while 1:
        cmd = input('1 : register new tless position\n2 : get tiago image and position\n\n')
        if cmd == "1":
            sub = Subscriber_tless("tiago_megapose")
        elif cmd == "2":
            sub = Subscriber_tiago("tiago_megapose")
        rospy.spin()

if __name__ == '__main__':
    listener()


#topic type image : sensor_msgs/Image
#topics mocap types : optitrack_ros/or_pose_estimator_state