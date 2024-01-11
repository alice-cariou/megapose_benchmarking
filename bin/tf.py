#!/usr/bin/env python

import utils

import rospy
import math
import tf2_ros
import geometry_msgs.msg

import os
import yaml
import argparse

import logging
logging.basicConfig()
logger = logging.getLogger('get_tless')
logger.setLevel(logging.INFO)

def tf(ex_name):
    rospy.init_node('tf2_test_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #now = rospy.Time.now()
    #if tfBuffer.can_transform('torso_lift_link', 'xtion_link', rospy.Time()):
    #    print('yay')
    #listener.waitForTransform('torso_lift_link', 'xtion_link', rospy.Time(), rospy.Duration(4.0))
    #(trans,rot) = listener.lookupTransform('torso_lift_link', 'xtion_link', now)
    trans = tfBuffer.lookup_transform('torso_lift_link', 'xtion_rgb_frame', rospy.Time(), rospy.Duration(1.0))
    print(trans.transform)
    pos = trans.transform.translation
    att = trans.transform.rotation

    content = {'tf_camera': {'pos': {'x': pos.x,'y': pos.y,'z' : pos.z},'quaternion':{'qw': att.w,'qx': att.x,'qy': att.y,'qz': att.z}}}
    utils.yaml_manager(ex_name, 'tf_camera', 'details.yaml', content)



def main():
    parser = argparse.ArgumentParser('Get object results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()
    if not args.name:
        logger.error('Please provide an example name : --name <example_name>')
        return

    tf(args.name)

if __name__ == '__main__':
    main()
