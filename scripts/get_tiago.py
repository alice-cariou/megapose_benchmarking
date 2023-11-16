#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from optitrack_ros.msg import or_pose_estimator_state

from cv_bridge import CvBridge, CvBridgeError
import cv2
import yaml

import logging
logging.basicConfig()
logger = logging.getLogger('get_tiago')
logger.setLevel(logging.INFO)

bridge = CvBridge()

class Subscriber_tiago:
    def __init__(self, name):
        rospy.init_node(name)
        print(0)
        self.sub2 = rospy.Subscriber("/xtion/rgb/images_raw", Image, lambda data: self.callback_image(data))
        #self.sub1 = rospy.Subscriber("/optitrack/bodies/tiago_base", or_pose_estimator_state, lambda data: self.callback_pos_tiago(data))


    def callback_pos_tiago(self, data):
        rospy.loginfo("I heard %s",data)
        if data.pos != []:
            self.sub1.unregister()
            print(data)
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
    sub = Subscriber_tiago("tiago_infos")
    rospy.spin()

def main():
    listener()

if __name__ == '__main__':
    main()
