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
To run the demonstration, do
```bash
$ cd ~/ros_ws
$ . baxter.sh
$ rosrun baxter_data_acquisition XXX
```
tbd
