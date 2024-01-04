import yaml
import os
import numpy as np
from statistics import mean, median

def get_results():
    ex_dir = os.path.dirname(os.path.realpath(__file__))+'/../tiago/'
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return

    transform_list = []

    for direct in os.walk(ex_dir):
        if 'results.yaml' in direct[2]:
            transform_list.append(add_result(direct[0]))
    
    return transform_list


def add_result(direct):
    yfile = direct + '/results.yaml'
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        trans = data['transformation from mocap to megapose']['translation']
        x, y, z = trans['x'], trans['y'], trans['z']
        rot = data['transformation from mocap to megapose']['rotation']
        rx, ry, rz = rot['x'], rot['y'], rot['z']

        return (np.array([abs(x), abs(y), abs(z)]), np.array([abs(rx), abs(ry), abs(rz)]))

def infos_transform(res,func):
    """apply func to each of res elements"""
    new_translation = [func([el[0][0] for el in res]),func([el[0][1] for el in res]),func([el[0][2] for el in res])]
    new_rotation = [func([el[1][0] for el in res]),func([el[1][1] for el in res]),func([el[1][2] for el in res])]
    return (new_translation, new_rotation)

def main():
    res = get_results() #res is a tuple2 list of transforms
    average = infos_transform(res,mean)
    mini = infos_transform(res,min)
    maxi = infos_transform(res,max)
    mediane = infos_transform(res,median)

if __name__ == '__main__':
    main()