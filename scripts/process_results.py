#!/usr/bin/env python

import utils
import yaml
import os
import numpy as np
from statistics import mean, median

def get_results():
    '''gathers all results'''
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
    name = direct.split('/')[-1]
    yfile = direct + '/results.yaml'
    with open(yfile, 'r') as f:
        data = yaml.safe_load(f)

        trans = data['transformation from mocap to megapose']['translation']
        x, y, z = trans['x'], trans['y'], trans['z']
        rot = data['transformation from mocap to megapose']['rotation']
        rx, ry, rz = rot['x'], rot['y'], rot['z']

        return (np.array([abs(x), abs(y), abs(z)]), np.array([abs(rx), abs(ry), abs(rz)]), name)
    return None

def infos_transform(res, func):
    """applies func to each of res elements"""
    new_translation = [func([el[0][0] for el in res]), func([el[0][1] for el in res]), func([el[0][2] for el in res])]
    new_rotation = [func([el[1][0] for el in res]), func([el[1][1] for el in res]), func([el[1][2] for el in res])]
    new = (new_translation, new_rotation)
    write_results(new, func.__name__)
    return new

def find_transform_utile(el):
    return el[0][0]+el[0][1]+el[0][2]

def find_transform(res,func):
    """finds func in examples"""
    found = func(find_transform_utile(el) for el in res)
    for i in range(len(res)):
        if find_transform_utile(res[i]) == found:
            write_results(res[i], func.__name__, res[i][2])
            return res[i], res[i][2]
    return -1


def write_results(res, name, i= None):
    '''writes res in all_results.yaml'''
    optional_content = {} if i == None else {'example_name': i}
    content = {'translation': {'x': float(res[0][0]),'y': float(res[0][1]),'z' : float(res[0][2])},'rotation':{'x': float(res[1][0]),'y': float(res[1][1]),'z': float(res[1][2])}}
    optional_content.update(content)
    full_content = {name: optional_content}
    print(full_content)

    yml_file = os.path.dirname(os.path.realpath(__file__))+'/../all_results.yaml'
    data = {}
    if os.path.exists(yml_file):
        with open(yml_file, 'r') as f:
            data = yaml.safe_load(f)
            if name in data:
                cmd = input("there are already "+name+" data in this yaml file. Do you wish to override them ? y/n ")
                cmd.lower()
                if cmd not in ('y','yes'):
                    print("aborting")
                    return
            data.update(full_content)
    with open(yml_file, 'w') as f:
        if not data:
            data = full_content
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    
    print('wrote results in all_results.yaml')

def main():
    res = get_results() #res is a tuple2 list of transforms + name

    average = infos_transform(res, mean)
    mediane = infos_transform(res, median)
    mini,i = find_transform(res, min)
    maxi,i = find_transform(res, max)

if __name__ == '__main__':
    main()