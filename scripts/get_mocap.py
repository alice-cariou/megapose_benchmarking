#!/usr/bin/env python

import utils

import rospy
from sensor_msgs.msg import Image
from optitrack_ros.msg import or_pose_estimator_state

import os
import yaml
import argparse

class Subscriber_mocap:
    def __init__(self, name, ex_name):
        rospy.init_node(name)
        self.ex_name = ex_name

        self.sub2 = rospy.Subscriber("/optitrack/bodies/tiago_torso_lift_link", or_pose_estimator_state, lambda data: self.callback_pos_tiago(data))
        self.sub1 = rospy.Subscriber("/optitrack/bodies/plank_gepetto", or_pose_estimator_state, lambda data: self.callback_plank(data))

    def callback_plank(self, data):
        rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub1.unregister()
            pos = data.pos[0]
            att = data.att[0]
            content = {'mocap_M_object': {'pos': {'x': pos.x,'y': pos.y,'z' : pos.z},'quaternion':{'qw': att.qw,'qx': att.qx,'qy': att.qy,'qz': att.qz}}}
            utils.yaml_manager(self.ex_name, 'mocap_M_object', 'details.yaml', content)

    def callback_pos_tiago(self, data):
        rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub2.unregister()
            pos = data.pos[0]
            att = data.att[0]
            content = {'mocap_M_tiago': {'pos': {'x': pos.x,'y': pos.y,'z' : pos.z},'quaternion':{'qw': att.qw,'qx': att.qx,'qy': att.qy,'qz': att.qz}}}
            utils.yaml_manager(self.ex_name, 'mocap_M_tiago', 'details.yaml', content)

def listener(ex_name):
    sub = Subscriber_mocap("tless_infos", ex_name)
    rospy.spin()

def main():
    parser = argparse.ArgumentParser('Get object results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()
    if not args.name:
        print('Please provide an example name : --name <example_name>')
        return

    listener(args.name)

if __name__ == '__main__':
    main()
