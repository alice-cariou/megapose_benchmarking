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
    def __init__(self, name, ex_name):
        rospy.init_node(name)
        self.ex_name = ex_name
        self.sub1 = rospy.Subscriber("/optitrack/bodies/plank_gepetto", or_pose_estimator_state, lambda data: self.callback_pos_wand(data))

    def callback_pos_wand(self, data):
        #rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub1.unregister()
            pos = data.pos[0]
            att = data.att[0]
            #self.sub1 = None
            test = {'object': {'pos': {'x': pos.x,'y': pos.y,'z' : pos.z},'att':{'qw': att.qw,'qx': att.qx,'qy': att.qy,'qz': att.qz}}}
            print(test)
            yml_path = os.path.dirname(os.path.realpath(__file__))+'/../tiago/{self.ex_name}}'
            if not os.path.exists(yml_path):
                os.makedirs(yml_path, exist_ok=True)
            if os.path.exists(f'{yml_path}+/details.yaml'):
                with open(f'{yml_path}+/details.yaml', 'r') as f:
                    data = yaml.safe_load(f)
                    if data['object']:
                        cmd = input("there are already objetct data in this yaml file. Do you wish to override them ? y/n ")
                        cmd.lower()
                        if cmd not in ('y','yes'):
                            logger.info("aborting")
                            return
            with open(f'{yml_path}+/details.yaml', 'w') as f:
                data = yaml.dump(test, f, default_flow_style=False)

def listener(ex_name):
    sub = Subscriber_tless("tless_infos", ex_name)
    rospy.spin()

def main():
    parser = argparse.ArgumentParser('Get object results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()
    if not args.name:
        logger.error('Please provide an example name : --name <example_name>')
        return

    listener(args.name)

if __name__ == '__main__':
    main()
