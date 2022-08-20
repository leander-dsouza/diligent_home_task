# Diligent Home Task

![Python](https://img.shields.io/badge/-Python-black?style=plastic&logo=Python)
![ROS](https://img.shields.io/badge/-ROS-22314E?style=plastic&logo=ROS)

A ROS1 package designed to tackle the Diligent Robotics Home task described as follows:

```
We have a robot that can drive around. At any point in time the position of the robot on the map can be represented by three numbers (x, y and theta), known as a 2D pose. We want to create a test simulation that will send the robot to a set of poses in a particular shape. Write a function in Python 3 that will produce a list of 2D poses that form an equilateral polygon:

The function takes three arguments:

* an integer specifying the number of sides of the polygon.
* the length of all sides of the polygon.
* the current 2D pose of the robot (optional, if not passed should assume the robot is at 0, 0, 0).
* The orientation of the waypoints is up to you.

Additionally create a script that will:

* Take in the number of sides and length as arguments.
* Get the pose of the robot from tf (you can assume there is a “map” frame and a “base_link” frame.
* Compute the list of 2D waypoints from the function you developed.
* Send the robot to each waypoint in succession (you can assume there is a “/move_base” action).
```

## Tutorial

* Two ROS packages are included in the submission - `atreus` and `diligent_home_task`. The former is just a testing platform that has a four wheeled differential drive robot with `move_base` support, while the latter is used to solve the required task.

* Install all the required dependencies at the root of the workspaces:

	  rosdep install --from-paths src --ignore-src -r -y && \
	  sudo apt install python3-pydocstyle

* Build your workspace and source it:

	  catkin build atreus diligent_home_task && source devel/setup.bash

* Test the package against coding and documentation standards, along with unit tests for the graded function - `compute_waypoints`:

	  catkin test diligent_home_task

* Launch the testing setup:

	  roslaunch atreus testing_setup.launch

	This will launch `navfn` as the global planner and `teb` as the local planner.

* In another terminal, source your workspace and optionally run the help command for usage:

	  rosrun diligent_home_task polygonal_path.py -h

	  usage: polygonal_path.py [-h] [-n NUM_SIDES] [-l LENGTH]

	  Publish Regular Polygonal Paths

	  optional arguments:
	    -h, --help            show this help message and exit
	    -n NUM_SIDES, --num_sides NUM_SIDES
	                          Number of sides of the polygon
	    -l LENGTH, --length LENGTH
	                          Length of the polygon

	Example:

	The node assumes default values for the number of sides (6) and length (10.0) if not specified.

	  rosrun diligent_home_task polygonal_path.py -n 5 -l 7.0

###### 💾 EOF
