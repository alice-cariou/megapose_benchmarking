# Test_set_megapose

Benchmarking of [megapose](https://github.com/agimus-project/happypose) using Tiago, objects of the [tless dataset](http://cmp.felk.cvut.cz/t-less/v) and mocap.

# Before use

That shouldn't be an issue if you used megapose before, but make sure you have a HAPPYPOSE_DATA_DIR environment variable (if you are using an older version of the project, MEGAPOSE_DATA_DIR will also work).

# Using this repository

You can find the benchmarking data in the tiago directory. Each example will have its results in a yaml file named like it, alongside details and informations about the example.  
If you wish to create new tests yourself, or run megapose on the existing tests, you can find the scripts that I used in the scripts directory.  
The following is about how to use the scripts.

## create a test using the mocap

### on optitrack

If it is the first time you use optitrack, you might need to setup a few things [here](https://wiki.laas.fr/robots/PR2/Mocap)  
Otherwise, you can already do :  
`cd openrobots_ws`  
`source setup-env.sh`  
`optitrack-ros -b`  
`rosaction call /optitrack/connect '{host: "muybridge", host_port: "1510", mcast: "239.192.168.30", mcast_port: "1511"}'`

You can now get the position of tiago, (I used the position of torso_lift_link), and the position of the object (I used the plank_gepetto):  
`./get_mocap.py --name <example_name>`

### on tiago

On a new terminal :  
`source setup_tiago.sh`

You should now have access to the topics regarding tiago.  
You can now get the image and the position of the camera with:  
`./get_tiago_infos.py --name <example_name>`

### afterwards

You can now use the informations you gathered and transform them to have a position and rotation comparable to megapose outputs :  
`./transform.py --name <example_name>`

After that, you should have a new example in the tiago directory, containing the data you just got.

## using megapose

To test a new example with megapose, you will need to :  

Create an inputs file for megapose, with the name of the example directory, the name of the object, and the coordinates of the bounding box around the object to detect. (temporary solution hopefully)
`./screate_inputs_file.py --name <example_name> --object <object_name> x1 y1 x2 y2`  
an example of use for this script would be :  
`./screate_inputs_file.py --name 004 --object tless23 300 286 425 336`

Create the megapose example using the informations you have in this directory :  
`./create_megapose_example.py --name <example_name>`

You can now run megapose on this new example using :  
`python -m happypose.pose_estimators.megapose.scripts.run_inference_on_example <example_name> --run-inference --vis-outputs --vis-detections`  
If you are not familiar with happypose, you might need to check its repository to configure your environment correctly : [happypose](https://github.com/agimus-project/happypose)

After that, you should be able to access the outputs of megapose for this example with :  
`./get_outputs_megapose.py --name <example_name>`

## comparing the results

You can find the results of the megapose detection and the mocap detection in the new example directory, in the yaml file with the corresponding name.


things to update :
results : first observation : errors in multiple directions, not always the same way, so probably not a mesuring issue
still waiting for more examples to be sure

change doc about how to use the scripts : change names and more
talk about the project organization
talk about how to see the results