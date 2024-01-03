#!/usr/bin/env python

import os
import argparse

def main():
    parser = argparse.ArgumentParser('Get megapose results')
    parser.add_argument('--name', type=str, help="name of the example dir")
    parser.add_argument('--object', type=str, help="name of the object to detect")
    parser.add_argument("x1", type=str)
    parser.add_argument("y1", type=str)
    parser.add_argument("x2", type=str)
    parser.add_argument("y2", type=str)

    args = parser.parse_args()

    content = '[{"label": "'+args.object+'", "bbox_modal": ['+args.x1+', '+args.y1+', '+args.x2+', '+args.y2+']}]\n'
    dir_name = os.path.dirname(os.path.realpath(__file__))+'/../../tiago/'+args.name
    if not os.path.exists(dir_name):
        os.makedirs(dir_name, exist_ok=True)
    if os.path.exists(dir_name+'/object_data.json'):
        cmd = input("there is already data in this file. Do you wish to override it ? y/n ")
        cmd.lower()
        if cmd not in ('y','yes'):
            logger.info("aborting")
            return
    with open(dir_name+"/object_data.json", "w") as f:
        f.write(content)

if __name__ == '__main__':
    main()
