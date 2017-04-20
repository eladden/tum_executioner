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

First you need to run the **tum_ardrone** package (the [vision group package](http://wiki.ros.org/tum_ardrone) or the [Simlulated GPS package](https://github.com/eladden/tum_ardrone_with_SGPS) with the [tum_simulator](https://github.com/dougvk/tum_simulator)).

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
cd tum_simulator_ws
source devel/setup.bash
rosrun tum_executioner ExecNode
```

## How to use

### The map
In your tum_executioner src folder there's a file named map.txt. this map is to contain the waypoints of the plan.3 It is assumed that each building has 4 points around it, and the points in the plan need to be ordered accordingly. That is, the first four points are the four points of building number 1, the next are the four points for building number 2 ect.

For example, map.txt may contain the following:
```
-10 0 3 90
7 -14 3 0
17 -2 3 -90
7 14 3 180
7 14 3 0
-10 34 3 90
7 44 3 180
17 34 3 -90
```
The first four points (``-10 0 3 90``,``7 -14 3 0``,``17 -2 3 -90``,``7 14 3 180``) belong to building 1, the next four, belong to building 2.
Edit your map file to match your world.

### The Topic

This node publishes and listens to a topic **/ExecTop**. The message is in the following format
```
int8 building
int8 building_point
bool fileready
```

when building or building_point changes in **/ExecTop** the node will read the target off the map file (for instance, if the topic changes to building 2  building_point 3, the node will seek the map file for the 3rd point of the 2nd building). The node then gives the drones autopilot a command to fly to the target. 

Upon reaching the target the node will snap a picture from the drone camera and save it to **image.png** in your workspace.

After taking the picture the node will change fileready to **TRUE** as an indicator that the file is new.


## Additional work required 

I attempted in defining the number of buildings and the number of points-per-building in a dynamic configuration. I was not successful and do not have time to complete this, therefore at the moment these values are defined as internal parameters in ExecNode.cpp (lines 174-176). If you need more buildings and/or more/less points-per-bulding this needs to be redefined internally (and then the package built again).

It would be nice if these parameters can be tuned externally, and while running.

Also the name of the map file is supposed to be "map.txt" this is not a parameter and should you want a different file, you should define this filename as a parameter as well.




