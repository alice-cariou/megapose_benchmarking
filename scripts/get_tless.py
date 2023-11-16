#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from optitrack_ros.msg import or_pose_estimator_state

import os
import yaml

import logging
logging.basicConfig()
logger = logging.getLogger('get_tless')
logger.setLevel(logging.INFO)

class Subscriber_tless:
    def __init__(self, name):
        rospy.init_node(name)
        self.sub1 = rospy.Subscriber("/optitrack/bodies/plank_gepetto", or_pose_estimator_state, lambda data: self.callback_pos_wand(data))

    def callback_pos_wand(self, data):
        #rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub1.unregister()
            pos = data.pos[0]
            att = data.att[0]
            #self.sub1 = None
            test = {'objpos': {'pos': {'x': pos.x,'y': pos.y,'z' : pos.z},'att':{'qw': att.qw,'qx': att.qx,'qy': att.qy,'qz': att.qz}}}
            print(test)
            yml_path = os.path.dirname(os.path.realpath(__file__))+'/../tiago/003'
            if not os.path.exists(yml_path):
                os.makedirs(yml_path, exist_ok=True)
            with open(yml_path+'/003.yaml', 'w') as f:
                data = yaml.dump(test, f, default_flow_style=False)
            #store data in yaml in new dir

def listener():
    sub = Subscriber_tless("tless_infos")
    rospy.spin()

def main():
    listener()

if __name__ == '__main__':
    main()
