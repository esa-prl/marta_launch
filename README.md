# marta_launch 

## Overview

This package contains launch files and tests that relate to the whole system of the MARTA rover.
**Keywords:** marta, launch

### License

The source code is released under a [TODO: Add License]().

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.com**

The PACKAGE NAME package has been tested under [ROS2] Eloquent and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies
The dependencies are dependant on which launch configuration of the rover you want to use.

#### Building

To build from source, clone the latest version from this repository into your ros2 workspace and compile the package using

	cd ros2_ws/src
	git clone https://github.com/esa-prl/marta_launch
	cd ../
	colcon build


### Tests
#### System Test
This test simulates the input of a gamepad and propagates the command through the gamepad_parser, locomotion_manager and locomotion_mode.

Run the system test with:

    cd ros2_ws/src/marta_launch/test
	launch_test test_system.py


## Usage

Run any of the provided launch files with:

	ros2 launch [FILE_NAME].launch.py

## Launch files


* **dummy_control.launch.py:**  

* **simple_simulation.launch.py** Launches all nodes to control a visualization of the robot in RVIZ2.

* **simulation_control.launch.py** Launches a simulation of the rover in Gazebo and the nodes to control it via gamepad.


## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
