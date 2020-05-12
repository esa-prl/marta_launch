# Setup

## Navigation

Clone the nav2 stack in a seperate workspace
```
mkdir rover_wss/nav2_ws/src

cd rover_wss/nav2_ws/sr
```

Clone the **forked** repository:

```
git clone https://github.com/esa-prl/navigation2
```

Checkout the following branch:

```
git checkout feature/tuning_params
```

Check dependencies:

```
cd ..

rosdep install -y -r -q --from-paths src --ignore-src --rosdistro eloquent
```

build the workspace w/ colcon.


## Gazebo Plugins
*Note: I am not sure if the feature is wanted by the core devs so this might change soon.*

In rover_wss/ros2_ws/src:

Clone the gazebo plugins:
```
git clone https://github.com/esa-prl/gazebo_ros_pkgs
```

Checkout the following branch
```
git checkout feature/seperate_child_frame_name
```

Build the workspace w/ colcon but without gazebo_ros as it makes troubles to compile and is not necessary.
```
colcon build --packages-ignore gazebo_ros
```

# Usage
Navigate to ros2_ws and source environment

First nav2_ws
```
source ../nav2_ws/install/setup.zsh
```

Then ros2_ws
```
source install/setup.zsh
```

Launch the nodes:
```
ros2 launch marta_launch nav2_marta_simulation_launch.py
```

This should launch RVIZ2 and Gazebo. If Gazebo crashes launch it in a seperate terminal with:

```
ros2 launch gazebo_ros gazebo.launch.py
```

In RVIZ click **STARTUP** in the bottom left. This starts the navigation nodes using the lifecycle manager, which loads the map in RVIZ.

You can activate the robot model in RVIZ for additional SWAG.

On the gamepad first activate the simple locomotion mode by clicking **X**.

Then, activate the navigation to a waypoint by clicking **BACK**

If the robot stops (don't know yet why), simply click **BACK** again and it will continue.

You can navigate away from the waypoint using the joystick and repeat the above.