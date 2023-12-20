from_tf_to_mocap = pin.SE3(np.array([[0,0,1],[1,0,0],[0,1,0]]),np.array([0,0,0]))

mocap_M_torso_lift_link = get_pin_SE3(ex_dir, 'details.yaml', 'base_robot')

torso_lift_link_M_mocap = from_tf_to_mocap*mocap_M_torso_lift_link.inverse()
quat_torso_lift_link_M_mocap = pin.SE3ToXYZQUAT(torso_lift_link_M_mocap)

quat_mocap_M_obj = pin.SE3ToXYZQUAT(mocap_M_obj)
quat_mocap_M_torso = pin.SE3ToXYZQUAT(mocap_M_torso_lift_link)

'''
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
            t3.transform.rotation.z = quat_torso_lift_link_M_mocap[6]'''
'''
            t4 = geometry_msgs.msg.TransformStamped()
            t4.header.frame_id = "torso_lift_link"
            t4.header.stamp = rospy.Time.now()
            t4.child_frame_id = "rob_M_cam"
            t4.transform.translation.x = quat_rob_M_cam[0]
            t4.transform.translation.y = quat_rob_M_cam[1]
            t4.transform.translation.z = quat_rob_M_cam[2]

            t4.transform.rotation.w = quat_rob_M_cam[3]
            t4.transform.rotation.x = quat_rob_M_cam[4]
            t4.transform.rotation.y = quat_rob_M_cam[5]
            t4.transform.rotation.z = quat_rob_M_cam[6]
'''