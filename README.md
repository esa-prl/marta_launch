# MaRTA Launch

## Overview

This package contains launch files and tests that relate to the whole system of the MARTA rover.
**Keywords:** marta, launch

### License

The source code is released under a [GPLv3 license](https://www.gnu.org/licenses/gpl-3.0.en.html).

**Author: Miro Voellmy<br />
Affiliation: [European Space Agency](https://www.esa.int/)<br />
Maintainer: Miro Voellmy, miro.voellmy@esa.int**

The MaRTA Launch package has been tested under [ROS2] Foxy Fitzroy and Ubuntu 20.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

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

	ros2 launch marta_launch [FILE_NAME].launch.py

## Launch Files

You can check the configration arguments of a launch file with the following command:

	ros2 launch marta_launch [FILE_NAME].launch.py -s

### Master Launch Files
The launch files can be launched in isolation and should deliver the described functionality.

* **gamepad_rover.launch.py:** Launches the platform driver together with RVIZ2 to control MaRTA by gamepad. Needs to be run in a root shell (`sudo -sE`).

* **gamepad_simulation.launch.py:** Launches MaRTA in an empty gazebo world ready to be controlled by a gamepad.

* **navigation_simulation.launch.py:** Launches MaRTA together with the [navigation stack](https://navigation.ros.org/) and GUI in RVIZ, which allows to set waypoints. Make sure to switch the locomotion_mode to somple_rover_locomotion with a physical gamepad or the [gamepad_emulation](https://github.com/esa-prl/gamepad_emulation).

* **simple_simulation.launch.py** Launches MaRTA as a visualization of the robot in RVIZ2.

### Sub Launch Files
These launch files contain nodes that are commonly launch together. They are not intended to be launched alone, but to be instead included in master launch files.

* **locomotion.launch.py** Launches all locomotion control nodes including the gamepad.

* **nav2_bringup_launch.py** Launches the navigation stack without any MaRTA features.

* **nav2_navigation_launch.py** Launches navigation nodes of nav2.

* **nav2_localization_launch.py** Launches localization nodes of nav2.

* **platform_driver_ethercat.launch.py** Launches the platform driver node and brings it to active state. Needs to be run in a root shell (`sudo -sE`).

* **simulation.launch.py** Launches the simulation only.

## Bugs & Feature Requests

Please report bugs and request features using the github issue tracker.


[ROS2]: http://www.ros.org
