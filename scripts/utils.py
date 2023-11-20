#!/usr/bin/env python

import os
import yaml

import logging
logging.basicConfig()
logger = logging.getLogger('test_set_megapose')
logger.setLevel(logging.INFO)

def yaml_manager(ex_name, key_name, filename, content):
    yml_path = os.path.dirname(os.path.realpath(__file__))+f'/../tiago/{ex_name}'
    if not os.path.exists(yml_path):
        os.makedirs(yml_path, exist_ok=True)
    if os.path.exists(f'{yml_path}/{filename}'):
        with open(f'{yml_path}/{filename}', 'r') as f:
            data = yaml.safe_load(f)
            if data[key_name]:
                cmd = input(f"there are already {key_name} data in this yaml file. Do you wish to override them ? y/n ")
                cmd.lower()
                if cmd not in ('y','yes'):
                    logger.info("aborting")
                    return
    with open(f'{yml_path}+/details.yaml', 'w') as f:
        data = yaml.dump(content, f, default_flow_style=False)
    
    logger.info(f'wrote results in tiago/{ex_name}/{filename}')