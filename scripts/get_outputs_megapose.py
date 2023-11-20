#!/usr/bin/env python

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

    test = {'megapose': {'quaternion':{'x' : two[0][0], 'y' : two[0][1],'z' : two[0][2], 'w?' : two[0][3]},'pos' : {'x' : two[1][0] , 'y' : two[1][1] , 'z' : two[1][2]}}}
    
    ex_dir = f'{os.path.dirname(__file__)}/../tiago/{ex_name}/{ex_name}.yaml'
    if not os.path.exists(ex_dir):
        logger.error('Make sure the example you asked for exists in the tiago directory, and that the .yaml has the right name')
        return

    with open(ex_dir, 'r') as f:
        data = yaml.safe_load(f)
        if 'megapose' in data.keys():
            cmd = input("there are already megapose data in this yaml file. Do you wish to override them ? y/n ")
            cmd.lower()
            if cmd not in ('y','yes'):
                logger.info("aborting")
                return

        data.update(test)
    if not data:
        logger.error('Error while reading yml file')
        return
    with open(ex_dir, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    logger.info('Done')

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