# Test_set_megapose

Benchmarking of [megapose](https://github.com/agimus-project/happypose) using Tiago, objects of the [tless dataset](http://cmp.felk.cvut.cz/t-less/v) and mocap.

# Before use

That shouldn't be an issue if you used megapose before, but make sure you have a HAPPYPOSE_DATA_DIR environment variable (if you are using an older version of the project, MEGAPOSE_DATA_DIR will also work).

# Using this repository

You can find the benchmarking data in the tiago directory. Each example will have its results in a yaml file named like it, alongside details and informations about the example.  
If you wish to create new tests yourself, or run megapose on the existing tests, you can find the scripts that I used in the scripts directory.  
The following is about how to use the scripts.

## create a test using the mocap

For a new test for megapose, you will need to get :

The position of tiago, I used the position of torso_lift_link :  
`./get_tiago.py --name <example_name>`

The image :  
`./test_image.py --name <example_name>`

The position of the object you want to detect with megapose :  
`./get_tless.py --name <example_name>`  
I used the `plank_gepetto` topic in this script, but feel free to modify this if you want to use something else to detect your objects.

The transformation between torso_lift_link and xtion_rgb_frame :  
`./tf.py --name <example_name>`

You can now use the informations you gathered and transform them to have a position and rotation comparable to megapose outputs :  
`./transform.py --name <example_name>`

After that, you should have a new example in the tiago directory, containing the data you just got.

## using megapose

To test a new example with megapose, you will need to :  

Create an inputs file for megapose, with the name of the example directory, the name of the object, and the coordinates of the bounding box around the object to detect.  
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

You can find the results of the megapose detection and the mocap detection, in the new example directory, in the yaml file with the corresponding name.