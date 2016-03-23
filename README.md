# baxter_data_acquisition
Data acquisition with the baxter research robot for robot anomaly detection.

## Description of software
tbd

## How to install and use
The baxter data acquisition software is implemented as a ROS package.

### Install the package
The following steps are required to install the package:

1. If not already done, set up your baxter workstation as explained in the 
[baxter SDK wiki](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup).
1. Install the fix for the baxter_interface repository. (tbd)
1. Install the extension of the baxter_description repository. (tbd)
1. Clone, build and install the ROS package:
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ cd ~/ros_ws/src
$ git clone https://github.com/BRML/baxter_data_acquisition.git
$ cd ~/ros_ws
$ catkin_make
$ catkin_make install
```

### Run the data acquisition
To run an experiment, do
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ rosrun baxter_data_acquisition XXX
```
where `XXX` describes the experiment. (tbd)

### Run the data acquisition in simulation mode
To start up the simulation environment (Gazebo) and load the experiment, do
```bash
$ cd ~/ros_ws
$ . baxter.sh sim
$ roslaunch baxter_data_acquisition baxter.launch debug:=true
```
It turns out that using the debug mode makes Gazebo more reliable, see 
[link](http://answers.gazebosim.org/question/5115/on-startup-of-gazebo-i-get-intermittent-error/).

Note: If the Error `Exception [Master.cc:50] Unable to start server[Address already in use].` 
pops up, do
```bash
$ killall gzserver
```
before trying to start Gazebo again.

To run an experiment in simulation, in another terminal do
```bash
$ cd ~/ros_ws
$ . baxter.sh sim
$ rosrun baxter_data_acquisition XXX
```
where `XXX` describes the experiment as it does for the experiments in the 
real world.

