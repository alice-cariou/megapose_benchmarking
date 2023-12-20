#!/usr/bin/env python

import utils

import os
import yaml
import numpy as np
import transforms3d.quaternions as tf_quaternions
import math
import pinocchio as pin
import pdb

import argparse
import logging
logging.basicConfig()
logger = logging.getLogger('tranform')
logger.setLevel(logging.INFO)

def truc():
    def get_rob_matrix(ex_dir):
        yfile = ex_dir + '/details.yaml'
        if not os.path.exists(ex_dir):
            logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
            return
        with open(yfile, 'r') as f:
            data = yaml.safe_load(f)

            robot_pos = data['base_robot']['pos']
            x, y, z = robot_pos['x'], robot_pos['y'], robot_pos['z']
            robot_q = data['base_robot']['quaternion']
            qw, qx, qy, qz = robot_q['qw'], robot_q['qx'], robot_q['qy'], robot_q['qz']
        
        rob_position = np.array([x, y, z])
        rob_quaternion = np.array([qw, qx, qy, qz])
        return to_matrix(rob_quaternion, rob_position)

    def get_cam_matrix(ex_dir):
        yfile = ex_dir + '/details.yaml'
        if not os.path.exists(ex_dir):
            logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
            return
        with open(yfile, 'r') as f:
            data = yaml.safe_load(f)

            tf_pos = data['tf_camera']['pos']
            x1,y1,z1 = tf_pos['x'], tf_pos['y'], tf_pos['z']
            tf_q = data['tf_camera']['quaternion']
            qw1, qx1, qy1, qz1 = tf_q['qw'], tf_q['qx'], tf_q['qy'], tf_q['qz']

        tf_vector = np.array([x1, y1, z1])
        tf_quaternion = np.array([qw1, qx1, qy1, qz1])

        #
        #cam_radian = to_radian(tf_quaternion)
        #roll, pitch, yaw = cam_radian[2], cam_radian[0], cam_radian[1] #x y z to z x y
        #tf_quaternion = to_quat(np.array([roll,pitch,yaw]))
        #

        return to_matrix(tf_quaternion,tf_vector)

    def get_obj_matrix(ex_dir):
        yfile = ex_dir + '/details.yaml'
        if not os.path.exists(ex_dir):
            logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
            return
        with open(yfile, 'r') as f:
            data = yaml.safe_load(f)

            object_pos = data['object']['pos']
            x, y, z = object_pos['x'], object_pos['y'], object_pos['z']
            object_q = data['object']['quaternion']
            qw, qx, qy, qz = object_q['qw'], object_q['qx'], object_q['qy'], object_q['qz']

        obj_position = np.array([x, y, z])
        obj_quaternion = np.array([qw, qx, qy, qz])
        return to_matrix(obj_quaternion, obj_position)

def to_matrix(q,pos):
    a,b,c,d = q[0], q[1], q[2], q[3]

    r11 = a*a + b*b - c*c - d*d
    r12 = 2*b*c - 2*a*d
    r13 = 2*b*d + 2*a*c

    r21 = 2*b*c + 2*a*d
    r22 = a*a - b*b + c*c - d*d
    r23 = 2*c*d - 2*a*b

    r31 = 2*b*d - 2*a*c 
    r32 = 2*c*d + 2*a*b
    r33 = a*a - b*b - c*c + d*d
    
    return np.array([[r11,r12,r13,pos[0]],[r21,r22,r23,pos[1]],[r31,r32,r33,pos[2]],[0,0,0,1]])

def to_radian(q):
    #q = w,x,y,z
    #angles = roll,pitch,yaw
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    roll = math.atan2(sinr_cosp, cosr_cosp) #x axix

    sinp = math.sqrt(1 + 2 * (q[0] * q[2] - q[1] * q[3]))
    cosp = math.sqrt(1 - 2 * (q[0] * q[2] - q[1] * q[3]))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2 #y axix

    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]) #z axix
    cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll,pitch,yaw])

def to_quat(rad):
    cr = math.cos(rad[0] * 0.5)
    sr = math.sin(rad[0] * 0.5)
    cp = math.cos(rad[1] * 0.5)
    sp = math.sin(rad[1] * 0.5)
    cy = math.cos(rad[2] * 0.5)
    sy = math.sin(rad[2] * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w,x,y,z])

def normalize(i):
    i%= 2 * math.pi
    if (i>math.pi):
        i-=2 * math.pi
    return i

def inv_rot_matrix(mat):
    rotation_matrix = mat[0:3,0:3]
    translation = mat[0:3,3]

    translation = -np.transpose(rotation_matrix)@translation
    rotation_matrix = np.transpose(rotation_matrix)

    res_matrix = np.c_[rotation_matrix, translation]
    res_matrix = np.r_[res_matrix, [[0,0,0,1]]]

    return res_matrix

def get_tf(ex_dir):
    yfile = ex_dir + '/details.yaml'
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        robot_pos = data['base_robot']['pos']
        x, y, z = robot_pos['x'], robot_pos['y'], robot_pos['z']
        robot_q = data['base_robot']['quaternion']
        qw, qx, qy, qz = robot_q['qw'], robot_q['qx'], robot_q['qy'], robot_q['qz']

        tf_pos = data['tf_camera']['pos']
        z1, x1, y1 = tf_pos['x'], tf_pos['y'], tf_pos['z']
        tf_q = data['tf_camera']['quaternion']
        qw1, qx1, qy1, qz1 = tf_q['qw'], tf_q['qx'], tf_q['qy'], tf_q['qz']

    rob_position = np.array([x, y, z])
    rob_quaternion = np.array([qw, qx, qy, qz])

    tf_vector = np.array([x1, y1, z1])
    tf_quaternion = np.array([qw1, qx1, qy1, qz1])

    cam_position = rob_position + tf_vector

    roll, pitch, yaw = tf_quaternion[1],tf_quaternion[2], tf_quaternion[0] #x y z to z x y
    tf_quaternion = to_quat(np.array([roll,pitch,yaw]))

    rob_radian = to_radian(rob_quaternion)
    tf_radian = to_radian(tf_quaternion)
    new = list(map(normalize, rob_radian + tf_radian))
    cam_quaternion = to_quat(new)

    return cam_position, cam_quaternion


def get_obj_pos(ex_dir, name, pos, quat):
    #pos apres tf
    x1, y1, z1  = pos[0], pos[1], pos[2]
    qw1, qx1, qy1, qz1 = quat [0],  quat[1], quat[2],  quat[3]

    #obj
    yfile = ex_dir + '/details.yaml'
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        object_pos = data['object']['pos']
        x, y, z = object_pos['x'], object_pos['y'], object_pos['z']
        object_q = data['object']['quaternion']
        qw, qx, qy, qz = object_q['qw'], object_q['qx'], object_q['qy'], object_q['qz']

    obj_position = np.array([x, y, z])
    obj_quaternion = np.array([qw, qx, qy, qz])

    cam_position = np.array([x1, y1, z1])
    cam_quaternion = np.array([qw1, qx1, qy1, qz1])

    result_position = cam_position - obj_position

    cam_radian = to_radian(cam_quaternion)
    obj_radian = to_radian(obj_quaternion)
    new = list(map(normalize, cam_radian - obj_radian))
    result_quaternion = to_quat(new)
    
    result_quaternion = np.array([result_quaternion[0], result_quaternion[1], result_quaternion[2], result_quaternion[3]])

    content = {'mocap': {'quaternion': {'qw':float(result_quaternion[0]),'qx':float(result_quaternion[1]),'qy':float(result_quaternion[2]),'qz':float(result_quaternion[3])}, 'pos': {'x':float(result_position[0]),'y':float(result_position[1]),'z':float(result_position[2])}}}

    utils.yaml_manager(name, 'mocap', name+'.yaml', content)
    return result_quaternion, result_position

def get_megapose_matrix(ex_dir):
    yfile = ex_dir + '/'+ex_dir.split('/')[-1]+'.yaml'
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        mega_pos = data['megapose']['pos']
        x, y, z = mega_pos['x'], mega_pos['y'], mega_pos['z']
        megapose_q = data['megapose']['quaternion']
        qw, qx, qy, qz = megapose_q['qw'], megapose_q['qx'], megapose_q['qy'], megapose_q['qz']
    
    megapose_position = np.array([x, y, z])
    megapose_quaternion = np.array([qw, qx, qy, qz])
    return to_matrix(megapose_quaternion, megapose_position)

def get_matrix(ex_dir, el): #el = 'tf_camera' | 'base_robot' | 'object'
    yfile = ex_dir + '/details.yaml'
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        robot_pos = data[el]['pos']
        x, y, z = robot_pos['x'], robot_pos['y'], robot_pos['z']
        robot_q = data[el]['quaternion']
        qw, qx, qy, qz = robot_q['qw'], robot_q['qx'], robot_q['qy'], robot_q['qz']
    
    rob_position = np.array([x, y, z])
    rob_quaternion = np.array([qw, qx, qy, qz])
    return to_matrix(rob_quaternion, rob_position)

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

def main():
    #breakpoint()
    parser = argparse.ArgumentParser('Transformations')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()

    if not args.name:
        logger.error('Please provide an example name : --name <example_name>')
        return

    ex_dir = os.path.dirname(__file__)+'/../tiago/'+args.name

    #rob_matrix = get_rob_matrix(ex_dir)
    #cam_matrix = get_cam_matrix(ex_dir)
    #obj_matrix = get_obj_matrix(ex_dir)
    #megapose_matrix = get_megapose_matrix(ex_dir)

    #test = inv_rot_matrix(cam_matrix)@inv_rot_matrix(rob_matrix)@obj_matrix
    #print(inv_rot_matrix(megapose_matrix)@test)

    rob_M_cam = get_pin_SE3(ex_dir, 'details.yaml', 'tf_camera')
    cam_M_rob = rob_M_cam.inverse()

    mocap_M_rob = get_pin_SE3(ex_dir, 'details.yaml', 'base_robot')
    rob_M_mocap = mocap_M_rob.inverse()

    mocap_M_obj = get_pin_SE3(ex_dir, 'details.yaml', 'object')

    cam_M_obj = cam_M_rob*rob_M_mocap*mocap_M_obj

    megapose_cam_M_obj = get_pin_SE3(ex_dir, '000.yaml', 'megapose')

    quat_cam_M_obj = pin.SE3ToXYZQUAT(cam_M_obj)
    quat_megapose_cam_M_obj = pin.SE3ToXYZQUAT(megapose_cam_M_obj)

    content = {'mocap': {'quaternion': {'qw':float(quat_cam_M_obj[0]),'qx':float(quat_cam_M_obj[1]),'qy':float(quat_cam_M_obj[2]),'qz':float(quat_cam_M_obj[3])}, 'pos': {'x':float(quat_cam_M_obj[0]),'y':float(quat_cam_M_obj[1]),'z':float(quat_cam_M_obj[2])}}}



    #test.inverse()

if __name__ == '__main__':
    main()
