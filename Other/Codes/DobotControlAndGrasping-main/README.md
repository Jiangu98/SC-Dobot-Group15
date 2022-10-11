# Sensors and Control Dobot Project: Control and Grasping
Authors
Dean Ockenden    13213537 - Sensors & Control and Robotics
Joseph Seklawy   12578845 - Robotics Team member
Will Hoogervorst 00000000 - Robotics Team member
Chris William    13248164 - Sensors and Control

Each file has a brief description if it is a function or is commented throughout to ensure ease of following.

## Startup.m
The startup.m file adds all of the files in the current directory to the path. It also runs the robotics toolbox. You will need to run this file or manually add all folders in the directory to path. You should also add a line with the location of your Robotics toolbox or run it separately.

## Main File - Dean and Chris
The main file runs the code demonstrated in our progress video. Running this code will take the data from the stored image in the directory and extract the block locations from the image as shown in the figures. These locations will then be sent to the Dobot Magician through ROS which will ideally pickup and drop off the blocks at the desired locations.

## Detection - Chris
The ColouredBlobDetection function in the detection folder is used for identifying the objects in the rgb image.
The folder also contains some example images & bag files that were used for testing.

	A full explanation of the ColouredBlobDetection function:
	- To use this function, you must input an image and define colour and dimensional threshold values.
	- The function first separates the colour bands of the image, then displays them in a figure
	- It then applies masks to the colour bands based on the defined colour thresholds, and displays them in the same figure
	- Blobs in the masks of a size smaller than a defined minimum threshold are removed, and the masks are combined in such a way to output a more refined masks
	- These refined masks are then reanalysed, checking for colour intensity and the blob with the highest intensity is selected, and all others eliminated
	- The centroids of the chosen blobs of each colour are used in conjunction with defined width of the surface, and the defined relative Dobot position to translate the centroid values to real world distances measured in metres.

## DobotControl - Dean and Joseph
The DobotControl folder contains the Resolved Motion Rate Control function used to control the Dobot Magician within simulation. This function isn't used in our demo with the real robot, however, the DobotMagician library allows for the sending of joint values so this could be integrated with the physical robot if desired.

## Models - Dean
The Models folder contains all of the ply models that we used for our simulation. This includes the Dobot magician's joints.

## ObjectClasses - Dean
The ObjectClasses folder contains all of the items that can currently be picked up by our Dobot in Simulation. All of the objects are children of the item class which contains the object location and a destructor to remove the objects from the scene.

## GUI - Will, Joseph and Dean
The GUI folder was primarily built for our robotics assignment but could be adapted for use with the real robot and ROS. Much of the functionality within the GUI will be broken as the functions that are used are not pertinent to this project.

## RealRobot - Dean
The RealRobot folder contains a class for the Dobot robot and the MoveDobot function which allows for the easy population of ROS messages and publishers when sending a command to the robot.

## RobotMechanics - Dean
The RobotMechanics folder contains a number of functions that helped us to work with the Dobot in simulation and manipulate objects.





         
