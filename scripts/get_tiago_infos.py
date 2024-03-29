#!/usr/bin/env python

import utils

import rospy
import math
import tf2_ros
import geometry_msgs.msg

import os
import yaml
import argparse
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import sys

class Robot_Subscriber:
    '''subscriber which gets the image seen by Tiago, and store them in the {ex_name} directory'''
    def __init__(self, name, ex_name):
        bridge = CvBridge()
        rospy.init_node(name, anonymous=True)
        self.ex_name = ex_name
        tf(ex_name)
        self.sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.callback)

    def callback(self, image_msg):
        print("Subscribe images from topic /xtion/rgb/image_raw ...")
        cv_image = imgmsg_to_cv2(image_msg)
        path=os.path.dirname(os.path.realpath(__file__))+'/../tiago/'+self.ex_name+'/image_rgb.png'
        cv2.imwrite(path, cv_image)
        print("save image as "+path)
        self.sub.unregister()

def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),dtype=dtype, buffer=img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv


def tf(ex_name):
    '''gets the tf transformation from torso_lift_link to the rgb camera (xtion_rgb_frame), store it in details.yaml'''
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform('torso_lift_link', 'xtion_rgb_frame', rospy.Time(), rospy.Duration(1.0))
    print(trans.transform)
    pos = trans.transform.translation
    att = trans.transform.rotation

    content = {'tiago_M_cam': {'pos': {'x': pos.x,'y': pos.y,'z' : pos.z},'quaternion':{'qw': att.w,'qx': att.x,'qy': att.y,'qz': att.z}}}
    utils.yaml_manager(ex_name, 'tiago_M_cam', 'details.yaml', content)

def main():
    parser = argparse.ArgumentParser('Get object results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()
    if not args.name:
        print('Please provide an example name : --name <example_name>')
        return

    sub = Robot_Subscriber("image_subscriber", args.name)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")

if __name__ == '__main__':
    main()
