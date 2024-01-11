#!/usr/bin/env python

import utils

import rospy
import tf2_ros
import geometry_msgs.msg

import yaml
import argparse

import numpy as np
from scipy.spatial.transform import Rotation as R

class Robot_Subscriber:
    def __init__(self, name,ex_name):
        rospy.init_node("test", anonymous=True)
        tf('mocap_M_obj','cam_M_megapose',ex_name)

def tf(header_frame, child_frame,ex_name):
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform(header_frame, child_frame, rospy.Time(), rospy.Duration(1.0))
    pos = trans.transform.translation
    att = trans.transform.rotation

    quat = np.array([att.x,att.y,att.z,att.w])
    rot = R.from_quat(quat)
    euler = rot.as_euler('xyz', degrees=True)

    content = {'transformation from mocap to megapose': {'translation': {'x': pos.x,'y': pos.y,'z' : pos.z},'rotation':{'x': float(euler[0]),'y': float(euler[1]),'z': float(euler[2])}}}
    utils.yaml_manager(ex_name, None, 'results.yaml', content)

def main():
    parser = argparse.ArgumentParser('Results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()
    if not args.name:
        print('Please provide an example name : --name <example_name>')
        return

    sub = Robot_Subscriber("image_subscriber",args.name)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down!")

if __name__ == '__main__':
    main()