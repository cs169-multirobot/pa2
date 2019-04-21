# Programming Assignment 2

A program that makes the turtlebot move towards the source of the wifi signal, while avoiding obstacles, and stop when reached the target wifi range.

## Getting Started

This package has been tested on Ubuntu 16.04.

Set up your caktin workspace:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
```

### Install

Either move this package to the `src` folder in the catkin workspace or clone it from the source repository. Then build and source:

```
git clone https://github.com/cs169-multirobot/pa2.git
cd ..
catkin_make
```

### Run

To run the program and start the robot to move:

```
roslaunch pa2 robot.launch [target_strength:=negative value...]
```

To start or stop the robot:

```
rosservice call safe_start_stop "{}"
```
