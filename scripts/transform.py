#!/usr/bin/env python

import utils

import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
import transforms3d.quaternions as tf_quaternions
import transforms3d.affines as tf_affines

import argparse
import logging
logging.basicConfig()
logger = logging.getLogger('tranform')
logger.setLevel(logging.INFO)

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
        qw1, qz1, qx1, qy1 = tf_q['qw'], tf_q['qx'], tf_q['qy'], tf_q['qz']

    #base robot
    #x, y, z = 1.14303636551, 11.585395813, 0.778616011143
    #qw, qx, qy, qz = -0.964628219604,  -0.000127985069412, 0.00720087811351, 0.263515889645

    #tf camera
    #x1, y1, z1 = 0.305, 0.020, 0.069
    #qw1, qx1, qy1, qz1 = 0.004, 0.465, -0.008, 0.885
    #z1, x1, y1 = 0.305, 0.020, 0.069
    #qw1, qz1, qx1, qy1 = 0.004, 0.465, -0.008, 0.885

    initial_position = np.array([x, y, z])
    initial_quaternion = np.array([qw, qx, qy, qz])

    translation_vector = np.array([x1, y1, z1])

    new_position = initial_position + translation_vector

    rotation_quaternion = np.array([qw1, qx1, qy1, qz1])

    new_quaternion = Rotation.from_quat(initial_quaternion) * Rotation.from_quat(rotation_quaternion)

    new_quaternion = new_quaternion.as_quat()

    #print("position:", new_position)
    #print("quaternion:", new_quaternion)
    return new_position, new_quaternion

def get_obj_pos(ex_dir, name, pos, quat):
    #pos après tf
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

    #x, y, z = 1.109344244, 10.9125547409,  0.19463776052
    #qw, qx, qy, qz = 0.669533729553, 0.0292000863701, -0.0205167289823, 0.741923928261

    position1 = np.array([x, y, z])
    quaternion1 = np.array([qw, qx, qy, qz])

    position2 = np.array([x1, y1, z1])
    quaternion2 = np.array([qw1, qx1, qy1, qz1])

    translation_vector = position2 - position1

    rotation_quaternion = tf_quaternions.qmult(quaternion2, tf_quaternions.qinverse(quaternion1))

    rotation_quaternion = np.array([rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2], rotation_quaternion[3]])

    content = {'mocap': {'quaternion': {'qw':int(rotation_quaternion[0]),'qx':int(rotation_quaternion[1]),'qy':int(rotation_quaternion[2]),'qz':int(rotation_quaternion[3])}, 'pos': {'x':int(translation_vector[0]),'y':int(translation_vector[1]),'z':int(translation_vector[2])}}}

    utils.yaml_manager(name, 'mocap', f'{name}.yaml', content)

def main():
    parser = argparse.ArgumentParser('Transformations')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()

    if not args.name:
        logger.error('Please provide an example name : --name <example_name>')
        return

    ex_dir = f'{os.path.dirname(__file__)}/../tiago/{args.name}'

    pos, quat = get_tf(ex_dir)
    get_obj_pos(ex_dir, args.name, pos, quat)

if __name__ == '__main__':
    main()