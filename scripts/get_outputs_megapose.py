#!/usr/bin/env python

import utils

import os
import argparse
import logging
import yaml
from ast import literal_eval

logging.basicConfig()
logger = logging.getLogger('get_outputs_megapose')
logger.setLevel(logging.INFO)

def get_megapose_outputs(ex_name):
    """get the megapose results from an example and add them to .yaml of that example (in tiago/<ex_name>/<ex_name>.yaml)"""
    localdir = os.getenv('HAPPYPOSE_DATA_DIR')
    if localdir == None:
        localdir = os.getenv('MEGAPOSE_DATA_DIR')
    if localdir == None:
        logger.error('Missing environnement variable : HAPPYPOSE_DATA_DIR or MEGAPOSE_DATA_DIR')
        return
    localdir += f'/examples/{ex_name}/outputs/object_data.json'

    if not os.path.exists(localdir):
        logger.error('Make sure the example you asked for exists in your $HAPPYPOSE_DATA_DIR/examples')
        return

    with open(localdir, 'r') as f:
        res = f.read()
    if not res :
        logger.error('Error while reading the megapose outputs file')
        return

    res = res[1:-1]
    res = literal_eval(res)
    two = res['TWO']

    ex_dir = f'{os.path.dirname(__file__)}/../tiago/{ex_name}'
    if not os.path.exists(ex_dir):
        os.makedirs(ex_dir, exist_ok=True)
        #logger.error('Make sure the example you asked for exists in the tiago directory')
        #return

    content = {'megapose': {'quaternion': {'qw' : two[0][0], 'qx' : two[0][1], 'qy' : two[0][2], 'qz' : two[0][3]},'pos' : {'x' : two[1][0] , 'y' : two[1][1] , 'z' : two[1][2]}}}
    utils.yaml_manager(ex_name, 'megapose', 'details.yaml', content)

def main():
    parser = argparse.ArgumentParser('Get megapose results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()

    if not args.name:
        logger.error('Please provide an example name : --name <example_name>')
        return

    get_megapose_outputs(args.name)

if __name__ == '__main__':
    main()