#!/usr/bin/env python

import os
import yaml

def yaml_manager(ex_name, key_name, filename, content):
    yml_path = os.path.dirname(os.path.realpath(__file__))+'/../tiago/'+ex_name
    if not os.path.exists(yml_path):
        os.makedirs(yml_path)
    data = {}
    if os.path.exists(yml_path+'/'+filename):
        with open(yml_path+'/'+filename, 'r') as f:
            data = yaml.safe_load(f)
            if key_name in data:
                cmd = input("there are already "+key_name+" data in this yaml file. Do you wish to override them ? y/n ")
                cmd.lower()
                if cmd not in ('y','yes'):
                    print("aborting")
                    return
            data.update(content)
    with open(yml_path+'/'+filename, 'w') as f:
        if not data:
            data = content
        yaml.safe_dump(data, f, default_flow_style=False)
    
    print('wrote results in tiago/'+ex_name+'/'+filename)
