# tum_executioner
This is a package that is meant to execute a plan for the AR drone using TUM vision.

## Installation

### with catkin

``` bash
cd catkin_ws/src
git clone https://github.com/eladden/tum_executioner.git
cd ..
rosdep install tum_executioner
catkin_make
```

# How to start

## Required packages
First you need to run the *tum_ardrone* package (the [vision group package](http://wiki.ros.org/tum_ardrone) or the [Simlulated GPS package](https://github.com/eladden/tum_ardrone_with_SGPS) with the [tum_simulator](https://github.com/dougvk/tum_simulator)).

### Using the drone in the lab with TUM vision
launch the ardrone driver and  tum ardrone GUI

``` bash
roslaunch tum_ardrone ardrone_driver.launch
roslaunch tum_ardrone tum_ardrone.launch
```

### Using gazebo and the simulated GPS
launch gazebo and your world, then the tum ardrone GUI
``` bash
roslaunch cvg_sim_gazebo ardrone_testoffice.launch
roslaunch tum_ardrone tum_ardrone_2.launch
```
notice this launches `tum_ardrone_2.launch` where the parameters are a bit different (more on this subject in the Simulated GPS package documentation).


## Run the node
Now that the tum ardrone sate estimation, autopilot and GUI are working you can run the executing node
``` bash
rosrun tum_executioner ExecNode
```

## How to use
In your tum_executioner src folder there's a file named map.txt. this map is to contain the waypoints of the plan.3 It is assumed that each building has 4 points around it, and the points in the plan need to be ordered accordingly. That is, the first four points are the four points of building number 1, the next are the four points for building number 2 ect.

For example, map.txt may contain the following:
```
-10 0 3 90
7 -14 3 0
17 -2 3 -90
7 14 3 180
7 14 3 0



