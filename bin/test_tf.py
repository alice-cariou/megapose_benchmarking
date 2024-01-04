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

class FixedTFBroadcaster:

    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=10)
        ex_dir = os.path.dirname(__file__)+"/../tiago/004"

        #rotation&translation matrixes
        from_tf_to_mocap = pin.SE3(np.array([[0,0,1],[1,0,0],[0,1,0]]),np.array([0,0,0]))
        from_mocap_to_tf = from_tf_to_mocap.inverse()
        from_panda_to_tf = pin.SE3(np.array([[0,-1,0],[1,0,0],[0,0,1]]),np.array([0,0,0]))
        from_megapose_to_tf = pin.SE3(np.array([[0,0,1],[0,-1,0],[1,0,0]]),np.array([0,0,0]))
        test = pin.SE3(np.array([[0,0,-1],[0,-1,0],[-1,0,0]]),np.array([0,0,0]))
        from_megapose_to_mocap = pin.SE3(np.array([[0,-1,0],[-1,0,0],[0,0,1]]),np.array([0,0,0]))

        torso_lift_link_M_cam = get_pin_SE3(ex_dir, 'details.yaml', 'tf_camera')
        quat_torso_lift_link_M_cam = pin.SE3ToXYZQUAT(torso_lift_link_M_cam)
        cam_M_torso_lift_link = torso_lift_link_M_cam.inverse()

        mocap_M_torso_lift_link = get_pin_SE3(ex_dir, 'details.yaml', 'base_robot')
        torso_lift_link_M_mocap = mocap_M_torso_lift_link.inverse()

        mocap_M_obj = get_pin_SE3(ex_dir, 'details.yaml', 'object')

        cam_M_obj = cam_M_torso_lift_link*torso_lift_link_M_mocap*mocap_M_obj

        cam_M_megapose = from_megapose_to_tf*get_pin_SE3(ex_dir, '004.yaml', 'megapose')


        quat_cam_M_obj = pin.SE3ToXYZQUAT(cam_M_obj)
        quat_cam_M_megapose = pin.SE3ToXYZQUAT(cam_M_megapose)

        #translation_torso_lift_base_link = np.array([0.06,0,-0.99])


        torso_lift_link_M_mocap = mocap_M_torso_lift_link.inverse()
        quat_torso_lift_link_M_mocap = pin.SE3ToXYZQUAT(torso_lift_link_M_mocap)
        quat_mocap_M_obj = pin.SE3ToXYZQUAT(mocap_M_obj)
        quat_mocap_M_torso = pin.SE3ToXYZQUAT(mocap_M_torso_lift_link)

        obj_M_megapose = cam_M_obj.inverse()*cam_M_megapose
        quat_obj_M_megapose = pin.SE3ToXYZQUAT(obj_M_megapose)

        print(quat_cam_M_obj,quat_cam_M_megapose)
        #+0.022

        while not rospy.is_shutdown():
            rospy.sleep(0.1)

            t = geometry_msgs.msg.TransformStamped()
            t.header.frame_id = "origin_mocap"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "torso_lift_link"
            t.transform.translation.x = quat_mocap_M_torso[0]
            t.transform.translation.y = quat_mocap_M_torso[1]
            t.transform.translation.z = quat_mocap_M_torso[2]
            t.transform.rotation.w = quat_mocap_M_torso[3]
            t.transform.rotation.x = quat_mocap_M_torso[4]
            t.transform.rotation.y = quat_mocap_M_torso[5]
            t.transform.rotation.z = quat_mocap_M_torso[6]

            t2 = geometry_msgs.msg.TransformStamped()
            t2.header.frame_id = "origin_mocap"
            t2.header.stamp = rospy.Time.now()
            t2.child_frame_id = "mocap_M_obj"
            t2.transform.translation.x = quat_mocap_M_obj[0]
            t2.transform.translation.y = quat_mocap_M_obj[1]
            t2.transform.translation.z = quat_mocap_M_obj[2]

            t2.transform.rotation.w = quat_mocap_M_obj[3]
            t2.transform.rotation.x = quat_mocap_M_obj[4]
            t2.transform.rotation.y = quat_mocap_M_obj[5]
            t2.transform.rotation.z = quat_mocap_M_obj[6]

            t3 = geometry_msgs.msg.TransformStamped()
            t3.header.frame_id = "torso_lift_link"
            t3.header.stamp = rospy.Time.now()
            t3.child_frame_id = "origin_mocap"
            t3.transform.translation.x = quat_torso_lift_link_M_mocap[0]
            t3.transform.translation.y = quat_torso_lift_link_M_mocap[1]
            t3.transform.translation.z = quat_torso_lift_link_M_mocap[2]

            t3.transform.rotation.w = quat_torso_lift_link_M_mocap[3]
            t3.transform.rotation.x = quat_torso_lift_link_M_mocap[4]
            t3.transform.rotation.y = quat_torso_lift_link_M_mocap[5]
            t3.transform.rotation.z = quat_torso_lift_link_M_mocap[6]

            t5 = geometry_msgs.msg.TransformStamped()
            t5.header.frame_id = "torso_lift_link"
            t5.header.stamp = rospy.Time.now()
            t5.child_frame_id = "test_cam"
            t5.transform.translation.x = quat_torso_lift_link_M_cam[0]
            t5.transform.translation.y = quat_torso_lift_link_M_cam[1]+0.022 #from xtion_link to xtion_rgb_frame
            t5.transform.translation.z = quat_torso_lift_link_M_cam[2]

            t5.transform.rotation.w = quat_torso_lift_link_M_cam[3]
            t5.transform.rotation.x = quat_torso_lift_link_M_cam[4]
            t5.transform.rotation.y = quat_torso_lift_link_M_cam[5]
            t5.transform.rotation.z = quat_torso_lift_link_M_cam[6]

            t6 = geometry_msgs.msg.TransformStamped()
            t6.header.frame_id = "test_cam"
            t6.header.stamp = rospy.Time.now()
            t6.child_frame_id = "test_megapose"
            t6.transform.translation.x = quat_cam_M_megapose[0]
            t6.transform.translation.y = quat_cam_M_megapose[1]
            t6.transform.translation.z = quat_cam_M_megapose[2]

            t6.transform.rotation.w = quat_cam_M_megapose[3]
            t6.transform.rotation.x = quat_cam_M_megapose[4]
            t6.transform.rotation.y = quat_cam_M_megapose[5]
            t6.transform.rotation.z = quat_cam_M_megapose[6]

            tfm = tf2_msgs.msg.TFMessage([t])
            tfm2 = tf2_msgs.msg.TFMessage([t2])
            tfm3 = tf2_msgs.msg.TFMessage([t3])
            #tfm4 = tf2_msgs.msg.TFMessage([t4])
            tfm5 = tf2_msgs.msg.TFMessage([t5])
            tfm6 = tf2_msgs.msg.TFMessage([t6])
            self.pub_tf.publish(tfm)
            self.pub_tf.publish(tfm2)
            #self.pub_tf.publish(tfm3)
            #self.pub_tf.publish(tfm4)
            self.pub_tf.publish(tfm5)
            self.pub_tf.publish(tfm6)

        
def get_quat_array(ex_dir, filename, el):#el = 'tf_camera' | 'base_robot' | 'object'
    yfile = f"{ex_dir}/{filename}"
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        pos = data[el]['pos']
        x,y,z = pos['x'], pos['y'], pos['z']
        q = data[el]['quaternion']
        qw, qx, qy, qz = q['qw'], q['qx'], q['qy'], q['qz']
    return np.array([x,y,z,qw,qx,qy,qz])

def get_pin_SE3(ex_dir, filename, el): #el = 'tf_camera' | 'base_robot' | 'object'
    yfile = f"{ex_dir}/{filename}"
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
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

def create_tfMessage(header_frame, child_frame, transform):
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

if __name__ == '__main__':
    rospy.init_node('test_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()