# Kinova Control
This package can be used to control a Kinova robot arm with Kortex, e.g. [Gen3](https://www.kinovarobotics.com/en/knowledge-hub/gen3-ultra-lightweight-robot). 
## Dependencies
1. [ROS Kinetic](http://wiki.ros.org/kinetic/Installation).
2. Install the [ROS Kortex package](https://github.com/Kinovarobotics/ros_kortex) in your catkin workspace.
## Usage
#### kinova_driver.py
This file contains functions that control the movements of the end-effector. There are three different sections in this
file: the gripper movement section, the cartesian mode movement section, and the joint mode movement section. This part 
of the package allows:
* Open and close the fingers
* Move the end-effector in a three-dimensional cartesian space, either via inputting the amount of displacement or by 
inputting the desired cartesian pose
* Get the current cartesian pose with respect to the base
* Change the pose of the robot by changing the joint angles
#### kinova_protection_zone.py
This file contains functions related to Kinova robots' protection zone functionality. 
The functions in this file can:
* Create a protection zone
* Delete a protection zone specified by the ProtectionZoneHandle
* Update a protection zone
* Get the current protection zone state
#### kinova_misc.py
This file contains miscellaneous functions that help the driver run smoothly, such as launching certain ros terminal 
commands.
#### create_zone_shape.py
The functions in this file are related to the [ZoneShape](https://github.com/Kinovarobotics/kortex/blob/master/api_python/doc/markdown/references/msg_Base_ZoneShape.md) msg type in the Kortex API.
These functions are able to:
* Obtain from the user specific information needed to create a ZoneShape instance, e.g. the dimensions
* Save the created ZoneShape instance to a file

## Examples
#### kinova_read_cartesian.py
This example saves the current cartesian position of the end-effector to a list when the key 's' is pressed, and saves
 the list to file when the key 'q' is pressed.
#### kinova_create_protection_zone.py
This example creates a protection zone, and allows the user to move the robot using the game controller in order to specify 
items such as the coordinates of the origin, the dimensions, etc.