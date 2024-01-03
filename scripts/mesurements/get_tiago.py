#!/usr/bin/env python

import utils

import rospy
from sensor_msgs.msg import Image
from optitrack_ros.msg import or_pose_estimator_state

from cv_bridge import CvBridge, CvBridgeError
import cv2
import yaml
import argparse

bridge = CvBridge()

class Subscriber_tiago:
    def __init__(self, name, ex_name):
        rospy.init_node(name)
        print(0)
        self.ex_name = ex_name
        self.sub1 = rospy.Subscriber("/optitrack/bodies/tiago_smth", or_pose_estimator_state, lambda data: self.callback_pos_tiago(data))
        self.sub2 = rospy.Subscriber("/xtion/rgb/images_raw", Image, lambda data: self.callback_image(data))

    def callback_pos_tiago(self, data):
        rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub1.unregister()

            pos = data.pos[0]
            att = data.att[0]
            content = {'base_robot': {'pos': {'x': pos.x,'y': pos.y,'z' : pos.z},'quaternion':{'qw': att.qw,'qx': att.qx,'qy': att.qy,'qz': att.qz}}}
            utils.yaml_manager(self.ex_name, 'base_robot', 'details.yaml', content)


    def callback_image(self, data):
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            cv2.imwrite('image_rgb.png', cv2_img)
            self.sub2.unregister()

def listener(ex_name):
    sub = Subscriber_tiago("tiago_infos", ex_name)
    rospy.spin()

def main():
    parser = argparse.ArgumentParser('Get tiago results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()
    if not args.name:
        print('Please provide an example name : --name <example_name>')
        return

    listener(args.name)

if __name__ == '__main__':
    main()
