#!/usr/bin/env python

import os
import sys
from pathlib import Path
import shutil
import argparse
from ast import literal_eval

import logging
logging.basicConfig()
logger = logging.getLogger('create_megapose_example')
logger.setLevel(logging.INFO)

def get_dir(name):
    '''creates a megapose example ready to be used from data in the repo'''
    #creates nexessary dirs
    frompath = f'{os.path.dirname(os.path.realpath(__file__))}/../tiago/{name}'
    if not os.path.exists(frompath):
        logger.error(f"the {frompath} directory does not exist")
        return
    datadir = os.environ.get('MEGAPOSE_DATA_DIR')
    if datadir == None:
        datadir = datadir = os.environ.get('HAPPYPOSE_DATA_DIR')
        if datadir == None:
            logger.error("you need to set your MEGAPOSE_DATA_DIR or HAPPYPOSE_DATA_DIR environment variable")
            return

    expath = f'{datadir}/examples/{name}'

    Path(expath).mkdir(parents=True,exist_ok=True)
    Path(f"{expath}/inputs").mkdir(exist_ok=True)
    Path(f"{expath}/meshes").mkdir(exist_ok=True)

    #creates necessary files
    if not os.path.exists(f"{frompath}/../camera_data.json"):
        logger.error(f"missing data file at {frompath}/../camera_data.json")
        return
    if not os.path.exists(f"{frompath}/object_data.json"):
        logger.error(f"missing data file at {frompath}/object_data.json")
        return
    
    shutil.copy(f"{frompath}/../camera_data.json", f"{expath}/camera_data.json")
    shutil.copy(f"{frompath}/object_data.json", f"{expath}/inputs/object_data.json")


    #gets the label of the object 
    with open(f"{frompath}/object_data.json", 'r') as f:
        res = f.read()
    if not res :
        logger.error(f'Error while reading {frompath}/obj_data.json')
        return
    res = res[1:-2]
    res = literal_eval(res)
    label = res['label']
    obj = label[-2:]

    #gets the right mesh according to the label
    Path(f"{expath}/meshes/{label}").mkdir(exist_ok=True)
    meshpath = f"{datadir}/bop_datasets/tless/models"
    if not os.path.exists(meshpath):
        logger.error("error in the meshes path. Did you download the tless dataset ?")
        return
    if not os.path.exists(f'{meshpath}/obj_0000{obj}.ply'):
        logger.error(f"error in the object name.\nExpected: tlessXX with XX between 01 and 30\nGot: {label}")
        return
    shutil.copy(f'{meshpath}/obj_0000{obj}.ply', f'{expath}/meshes/{label}')

    #copies image
    if not os.path.exists(f"{frompath}/image_rgb.png"):
        logger.error("missing image")
        return
    shutil.copy(f"{frompath}/image_rgb.png", f"{expath}/image_rgb.png")

    logger.info(f'created example at {expath}')

def main():
    parser = argparse.ArgumentParser('Get megapose results')
    parser.add_argument('--name', type=str)

    args = parser.parse_args()

    if not args.name:
        logger.error('Please provide an example name : --name <example_name>')
        return
    
    get_dir(args.name)

if __name__ == '__main__':
    main()
