#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation
import transforms3d.quaternions as tf_quaternions
import transforms3d.affines as tf_affines

def get_tf():
    #base robot
    x, y, z = 1.14303636551, 11.585395813, 0.778616011143
    qw, qx, qy, qz = -0.964628219604,  -0.000127985069412, 0.00720087811351, 0.263515889645

    #tf camera
    #x1, y1, z1 = 0.305, 0.020, 0.069
    #qw1, qx1, qy1, qz1 = 0.004, 0.465, -0.008, 0.885
    z1, x1, y1 = 0.305, 0.020, 0.069
    qw1, qz1, qx1, qy1 = 0.004, 0.465, -0.008, 0.885

    initial_position = np.array([x, y, z])
    initial_quaternion = np.array([qw, qx, qy, qz])

    translation_vector = np.array([x1, y1, z1])

    new_position = initial_position + translation_vector

    rotation_quaternion = np.array([qw1, qx1, qy1, qz1])

    new_quaternion = Rotation.from_quat(initial_quaternion) * Rotation.from_quat(rotation_quaternion)

    new_quaternion = new_quaternion.as_quat()

    print("position:", new_position)
    print("quaternion:", new_quaternion)
    return new_position, new_quaternion

def get_obj_pos(pos, quat):
    #pos après tf
    x1, y1, z1  = pos[0], pos[1], pos[2]
    qw1, qx1, qy1, qz1 = quat [0],  quat[1], quat[2],  quat[3]

    #obj
    x, y, z = 1.109344244, 10.9125547409,  0.19463776052
    qw, qx, qy, qz = 0.669533729553, 0.0292000863701, -0.0205167289823, 0.741923928261

    position1 = np.array([x, y, z])
    quaternion1 = np.array([qw, qx, qy, qz])

    position2 = np.array([x1, y1, z1])
    quaternion2 = np.array([qw1, qx1, qy1, qz1])

    translation_vector = position2 - position1

    rotation_quaternion = tf_quaternions.qmult(quaternion2, tf_quaternions.qinverse(quaternion1))

    rotation_quaternion = np.array([rotation_quaternion[0], rotation_quaternion[1], rotation_quaternion[2], rotation_quaternion[3]])

    print("translation:", translation_vector)
    print("quaternion:", rotation_quaternion)

def main():
    pos, quat = get_tf()
    print('\n')
    get_obj_pos(pos, quat)

if __name__ == '__main__':
    main()