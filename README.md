# Test_set_megapose

Benchmarking of [megapose](https://github.com/agimus-project/happypose) using Tiago, objects of the [tless dataset](http://cmp.felk.cvut.cz/t-less/v) and mocap.

# Using this repository

The results can be found in the all_results.yaml file, with the average, median, minimum and maximum results.

The benchmarking data is located in the `tiago` directory. Each example subdirectory has :
- a `results.yaml` file, containing the transformation between the mocap mesurement (our truth) and the megapose results
- the image as `image_rgb.png`
- an `object_data.json` file, containing the label and bbox needed for megapose in the right format
- a `details.yaml` with all the mesurements used

To create new tests, visualize the results or run megapose on the existing tests, scripts can be found in the `scripts` directory.
The following instructions describe how to use the scripts.

# Before use

In order to use some of these scripts, installations or configurations might be necessary.
- to use megapose relating scripts : [happypose](https://github.com/agimus-project/happypose)
- to make new mesurements :  [mocap](https://wiki.laas.fr/robots/PR2/Mocap)
- to visualize results : [rviz](#rviz-configuration)

## create a test using tiago and the mocap

Note on the following scripts : They were written to be used with python2.7 and ROS1 Melodic.

### on a computer connected to tiago

To use the following commands, you need to be able to connect to the mocap.
At the root of the repository :
```
source setup_tiago.sh
optitrack-ros -b
rosaction call /optitrack/connect '{host: "muybridge", host_port: "1510", mcast: "239.192.168.30", mcast_port: "1511"}'
```
You should then have access to the topics published by tiago, as well as the optitrack topics.
Everything is ready to make mesurements.

### tiago and object position in mocap

To make the following step easier, the position used to detect tiago's position corresponds to torso_lift_link in the robot model, on top of the robot, and is called tiago_torso_lift_link.

In addition, to detect the position of the object without modifying their apparence, plank_gepetto was used, it is a plexiglas plank with mocap captors on it.

The following script will get the position and the rotation of tiago_torso_lift_link and plank_gepetto and store it in the corresponding details.yaml file.

`./get_mocap.py --name <example_name>`

### tiago image and camera position

To get the image and the transform from torso_lift_link to the camera :

`./get_tiago_infos.py --name <example_name>`

## using megapose

Note on the following scripts : the were written for python3.

### create the input file an example

For a new megapose example, two things are necessary :
- an image
- an input file

An input file can be created with the name of the example directory, the name of the object, and the coordinates of the bounding box around the object to detect.

`./megapose_create_inputs_file.py --name <example_name> --object <object_name> x1 y1 x2 y2`

an example of use of this script would be :

`./megapose_create_inputs_file.py --name 004 --object tless23 300 286 425 336`

This will create an `object_data.json` file with the format megapose expects is to have.

### create a megapose example

A megapose example can be created using the informations you have in any `tiago` subdirectory :

`./megapose_create_example.py --name <example_name>`

This will create the example in the datadir you chose for happypose, ready to be used.

### running megapose

Megapose can now be run on this new example using :

`python -m happypose.pose_estimators.megapose.scripts.run_inference_on_example <example_name> --run-inference --vis-outputs`

### megapose outputs
  
After that, the outputs of megapose will be accessible with :

`./get_outputs_megapose.py --name <example_name>`

This will add megapose informations in the details.yaml of your example directory.
  

## comparing the results

## visualizing the results

To visualize the results with rviz and calculate the transformation from one to the other, rviz can be used.
```
source tiago_public_ws/devel/setup.bach
roslaunch tiago_description ac_show.launch
```
Then, the results can be visualized with :
`./process_mesurements.py --name <example_name>`

## calculating the transform between megapose and mocap

In another window:

`./process_tf_obj --name <example_name>`

This will calculate the transform between what was detected by megapose and the "truth" (mocap), and write in the results.yaml file of your example directory.

## get updated general results

Finally, if a new example was added, it can be taken into consideration in the all_results.yaml file :

`./process_results`

# rviz configuration

The installation is detailled [here](http://wiki.ros.org/Robots/TIAGo/Tutorials/Installation/InstallUbuntuAndROS)

A new workspace should then have been created (named tiago_public_ws in the tutorial)

You can now copy the `tiago/ac_show.launch` file in your `tiago_public_ws/src/tiago_robot/tiago_description/robots/` directory

then, each time you need to use it :
```
source tiago_public_ws/devel/setup.bach
roslaunch tiago_description ac_show.launch
```
