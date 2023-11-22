#!/usr/bin/env python  
import roslib
import rospy
from optitrack_ros.msg import or_pose_estimator_state

import tf
import turtlesim.msg

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    optipose = msg.pos[0]
    optiatt = msg.att[0]
    print(optipose)
    br.sendTransform((optipose.x + 6.23, optipose.y + 2.977, optipose.z + 0.404),
                     (optiatt.qx, optiatt.qy, optiatt.qz, optiatt.qw),
                     rospy.Time.now(),
                     "tiago_ref",
                     "map")

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/optitrack/bodies/tiago_smth', or_pose_estimator_state, handle_turtle_pose)
    rospy.spin()
