#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
import os
import yaml
import numpy as np
import pinocchio as pin
import math
import argparse

class FixedTFBroadcaster:
    '''gets mesurements in details.yaml and broadcasts them'''
    def __init__(self, name):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
        self.transform_array = []
        ex_dir = os.path.dirname(__file__)+"/../tiago/"+name

        from_megapose_to_tf = pin.SE3(np.array([[0,0,1],[0,-1,0],[1,0,0]]),np.array([0,0,0]))

        #cam_M_torso_lift_link
        torso_lift_link_M_cam = get_pin_SE3(ex_dir, 'details.yaml', 'tiago_M_cam')
        quat_torso_lift_link_M_cam = pin.SE3ToXYZQUAT(torso_lift_link_M_cam)
        cam_M_torso_lift_link = torso_lift_link_M_cam.inverse()

        #torso_lift_link_M_mocap
        mocap_M_torso_lift_link = get_pin_SE3(ex_dir, 'details.yaml', 'mocap_M_tiago')
        torso_lift_link_M_mocap = mocap_M_torso_lift_link.inverse()

        #mocap_M_obj
        mocap_M_obj = get_pin_SE3(ex_dir, 'details.yaml', 'mocap_M_object')
        quat_mocap_M_obj = pin.SE3ToXYZQUAT(mocap_M_obj)

        #cam_M_obj
        cam_M_obj = cam_M_torso_lift_link*torso_lift_link_M_mocap*mocap_M_obj
        quat_cam_M_obj = pin.SE3ToXYZQUAT(cam_M_obj)

        #cam_M_megapose
        cam_M_megapose = from_megapose_to_tf*get_pin_SE3(ex_dir, f'details.yaml', 'megapose__cam_M_object')
        quat_cam_M_megapose = pin.SE3ToXYZQUAT(cam_M_megapose)

        #mocap_M_torso_lift_link
        torso_lift_link_M_mocap = mocap_M_torso_lift_link.inverse()
        quat_torso_lift_link_M_mocap = pin.SE3ToXYZQUAT(torso_lift_link_M_mocap)
        quat_mocap_M_torso = pin.SE3ToXYZQUAT(mocap_M_torso_lift_link)

        while not rospy.is_shutdown():
            rospy.sleep(0.1)

            tfm = create_tfMessage("origin_mocap", "torso_lift_link", quat_mocap_M_torso)
            tfm2 = create_tfMessage("origin_mocap", "mocap_M_obj", quat_mocap_M_obj)
            tfm3 = create_tfMessage("torso_lift_link", "origin_mocap", quat_torso_lift_link_M_mocap)
            tfm4 = create_tfMessage("torso_lift_link", "cam", quat_torso_lift_link_M_cam)
            tfm5 = create_tfMessage("cam","cam_M_megapose", quat_cam_M_megapose)

            self.pub_tf.publish(tfm)
            self.pub_tf.publish(tfm2)
            #self.pub_tf.publish(tfm3)
            self.pub_tf.publish(tfm4)
            self.pub_tf.publish(tfm5)

def get_pin_SE3(ex_dir, filename, el): #el = 'tiago_M_cam' | 'mocap_M_tiago' | 'mocap_M_object'
    '''creates a pin.SE3 from mesurements in details.yaml'''
    yfile = f"{ex_dir}/{filename}"
    if not os.path.exists(ex_dir):
        print('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        pos = data[el]['pos']
        x, y, z = pos['x'], pos['y'], pos['z']
        q = data[el]['quaternion']
        qw, qx, qy, qz = q['qw'], q['qx'], q['qy'], q['qz']
    
    position = np.array([x, y, z])
    quaternion = np.array([qw, qx, qy, qz])
    return pin.SE3(pin.Quaternion(quaternion), position)

def create_tfMessage(header_frame, child_frame, transform): #transform is a pin.SE3ToXYZQUAT
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = header_frame
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame
    t.transform.translation.x = transform[0]
    t.transform.translation.y = transform[1]
    t.transform.translation.z = transform[2]
    t.transform.rotation.w = transform[3]
    t.transform.rotation.x = transform[4]
    t.transform.rotation.y = transform[5]
    t.transform.rotation.z = transform[6]

    return tf2_msgs.msg.TFMessage([t])

def main():
    parser = argparse.ArgumentParser('tf_rviz')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()
    if not args.name:
        print('Please provide an example name : --name <example_name>')
        return

    rospy.init_node('test_broadcaster')
    tfb = FixedTFBroadcaster(args.name)

    rospy.spin()

if __name__ == '__main__':
    main()