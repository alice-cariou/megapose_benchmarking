#!/usr/bin/env python
import rospy
import math
import tf2_ros
import geometry_msgs.msg

def main():
    rospy.init_node('tf2_turtle_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform('turtle2', 'turtle1', rospy.Time.now(), rospy.Duration(1.0))
    print(trans)

if __name__ == '__main__':
    main()