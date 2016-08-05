# The Baxter data acquisition framework
Data acquisition with the Baxter research robot for robot anomaly detection 
and movement prediction.
The framework heavily builds upon the 
[Baxter SDK](https://github.com/RethinkRobotics) and depends on the 
[depth_sensors](https://github.com/BRML/depth_sensors.git) ROS package as 
well as customized versions of 
[baxter_interface](https://github.com/BRML/baxter_interface.git) and
[baxter_common](https://github.com/BRML/baxter_common.git) from Rethink
Robotics.


## Description of software
In this [ROS](http://www.ros.org/) package several experiments with the 
[Baxter research robot](http://www.rethinkrobotics.com/research-education/) 
are implemented.
Those are intended for recording (internal) joint data (angles, torques,
accelerations, poses, anomalies) as well as (external) visual data with RGB 
and depth cameras.

All experiments can be run both on the real robot as well as in the 
[Gazebo](http://gazebosim.org/)-powered 
[Baxter simulator](http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator).

### Structure of the repository
```bash
.
|
+-- data/setup/        setup files for anomaly experiment
|
+-- launch/            ROS launch files
|
+-- nodes/             implementation of recorder ROS nodes
|
+-- scripts/           implementation of experiment ROS nodes
|
+-- share/images/      images for collision experiment
|
+-- src/                            python modules
|   +-- baxter_data_acquisition/    helper functions and settings
|   +-- control/                    custom controller and interpolator
|   +-- experiments/                implementation of experiments
|   +-- recorder/                   implementation of recorders
|
+-- srv/               custom ROS service definitions for recorder nodes
|
+-- urdf/              custom URDF describing the Baxter robot
```

## How to install and use
The Baxter data acquisition software is implemented as a ROS package.
It requires a development workstation with 
[Ubuntu 14.04](http://releases.ubuntu.com/14.04/) and 
[ROS Indigo](http://wiki.ros.org/indigo) installed.

> Note: If you have Ubuntu, ROS and and the Baxter SDK dependencies already 
> installed, you only need to perform steps 3, 5 and 6 to clone, install and 
> setup the Baxter data acquisition framework!

### Step 1: Install Ubuntu
Follow the standard Ubuntu Installation Instructions for 14.04 (Desktop).

### Step 2: Install ROS Indigo
Configure your Ubuntu repositories to allow "restricted," "universe," and 
"multiverse."

#### Setup your sources.list
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
```

#### Setup your keys
```bash
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```

#### Verify latest debians, install ROS Indigo Desktop Full and rosinstall
```bash
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop-full
$ sudo rosdep init
$ rosdep update
$ sudo apt-get install python-rosinstall
```

### Step 3: Create ROS workspace
```bash
$ mkdir -p ~/ros_baxter_daq_ws/src
$ source /opt/ros/indigo/setup.bash
$ cd ~/ros_baxter_daq_ws
$ catkin_make
$ catkin_make install
```

### Step 4: Install Baxter SDK-, Baxter simulator and data recording dependencies
```bash
$ sudo apt-get update
$ sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep ros-indigo-control-msgs ros-indigo-joystick-drivers
$ sudo apt-get install gazebo2 ros-indigo-qt-build ros-indigo-driver-common ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-ros-control ros-indigo-control-toolbox ros-indigo-realtime-tools ros-indigo-ros-controllers ros-indigo-xacro python-wstool ros-indigo-tf-conversions ros-indigo-kdl-parser
$ sudo apt-get install python-dev libhdf5-dev python-numpy
$ pip install h5py
$ sudo apt-get install libsnappy-dev
$ pip install python-snappy
```
To install the Microsoft Kinect V2 for Linux we need the 
[libfreenect2](https://github.com/OpenKinect/libfreenect2/blob/master/README.md#linux) 
library, which we will build from source and install into ~/freenect2:
```bash
$ sudo apt-get install libturbojpeg libopenni2-dev
$ git clone https://github.com/OpenKinect/libfreenect2.git ~/libfreenect2
$ cd ~/libfreenect2/depends
$ ./download_debs_trusty.sh
$ sudo dpkg -i debs/libglfw3*deb
$ sudo apt-get install -f
cd ~/libfreenect2
mkdir build
cd build
cmake .. -DENABLE_CXX11=ON -DCMAKE_INSTALL_PREFIX=$HOME/freenect2
make
make install
sudo cp ../platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```
To verify the installation, plug in the Kinect V2 into an USB 3 port and
run the `./bin/Protonect` test program.

### Step 5: Install this package and its dependencies
Using the [wstool](http://wiki.ros.org/wstool) workspace tool, you will 
checkout all required Github repositories into your ROS workspace source 
directory.
The [ROS bridge for the Kinect V2](https://github.com/code-iai/iai_kinect2#install) 
will be installed separately.
```bash
$ cd ~/ros_baxter_daq_ws/src
$ wstool init .
$ wstool merge https://raw.githubusercontent.com/BRML/baxter_rosinstall/master/baxter_daq.rosinstall
$ wstool update
$ cd iai_kinect2
$ source /opt/ros/indigo/setup.bash
$ rosdep install -r --from-paths .
$ cd ~/ros_baxter_daq_ws
$ catkin_make -DCMAKE_BUILD_TYPE="Release"
$ catkin_make install
```

### Step 6: Configure Baxter communication/ROS workspace
The [baxter.sh](http://sdk.rethinkrobotics.com/wiki/Baxter.sh) script is a 
convenient script which allows for intuitive modification of the core ROS 
environment components. 
This user edited script will allow for the quickest and easiest ROS setup.
Further information and a detailed description is available on the 
[baxter.sh](http://sdk.rethinkrobotics.com/wiki/Baxter.sh) page.

#### Download the baxter.sh script
```bash
$ cd ~/ros_baxter_daq_ws
$ wget https://github.com/RethinkRobotics/baxter/raw/master/baxter.sh
$ chmod u+x baxter.sh
```

#### Customize the baxter.sh script
Using your favorite editor, edit the baxter.sh shell script making the 
necessary modifications to describe your development workstation.

- Edit the `baxter_hostname` field to match the hostname of your Baxter 
robot.
- Edit **either** the `your_ip` **or** the `your_hostname` field to 
match the IP or hostname of your development workstation.
Only one of those fields can be active at a time.
The other variable should be commented out!

#### Initialize your SDK environment
```bash
$ cd ~/ros_baxter_daq_ws
$ . baxter.sh
```

#### Verify environment
To verify that all your changes are applied correctly, perform
```bash
$ env | grep ROS
```
The important fields at this point are

- **ROS_MASTER_URI** (this should now contain your robot's hostname)
- **ROS_IP** or **ROS_HOSTNAME** (this should now contain your development
workstation's ip address or hostname. The unused field should **not** be 
available!)


## Run the data acquisition
To run an experiment, initialize your SDK environment and `rosrun` the 
experiment.
That is, do
```bash
$ cd ~/ros_baxter_daq_ws
$ . baxter.sh
$ rosrun baxter_data_acquisition XXX
```
where `XXX` describes the experiment. 
Implemented experiments are:

- `collision.py`
- `anomaly.py`
- `handshake.py`
- `goal.py`

Use the `-h` command line option to learn more about the experiments and its
required and optional parameters.

Example:
```bash
$ cd ~/ros_baxter_daq_ws
$ . baxter.sh
$ rosrun baxter_data_acquisition anomaly.py -l left -a true -n 15
```
This will run 15 samples of the joint position anomaly experiment on Baxter's 
left arm with automatically induced anomalies.


## Run the data acquisition in simulation mode
To start up the simulation environment (Gazebo) and run an experiment, 
initialize your SDK environment in simulation mode, `roslaunch` the simulator
and data recorder convenience scripts and `rosrun` the experiment.
That is, in a terminal do
```bash
$ cd ~/ros_baxter_daq_ws
$ . baxter.sh sim
$ roslaunch baxter_data_acquisition simulation.launch
```
In another terminal do
```bash
$ cd ~/ros_baxter_daq_ws
$ . baxter.sh sim
$ roslaunch baxter_data_acquisition recorder.launch
```
And in a third terminal do
```bash
$ cd ~/ros_baxter_daq_ws
$ . baxter.sh sim
$ rosrun baxter_data_acquisition XXX
```
where `XXX` describes the experiment as it does for the experiments with the 
real Baxter robot.

Note: The `roslaunch` files have parameters to modify their behavior. Please
have a look at the files for more information.


### Experiment convenience launch file
There also are launch files that collect the three separate steps above into
one file.
This serves a two-fold purpose.
First, it is more convenient to start the whole experiment from a single 
terminal.
Second, it allows for aborting the experiment if a single ROS node died, 
increasing robustness of the data recording procedure.


### Known bugs and annoying peculiarities of Gazebo

- If the Error `Exception [Master.cc:50] Unable to start server[Address already in use].` 
pops up, do
```bash
$ killall gzserver
```
before trying to start Gazebo again.
- If the Error 
```
Error [Param.cc:181] Unable to set value [1,0471975511965976] for key[horizontal_fov]
Error [Param.cc:181] Unable to set value [0,100000001] for key[near]`
```
pops up, do (see [link](http://answers.ros.org/question/199401/problem-with-indigo-and-gazebo-22/)) 
```bash
$ export LC_NUMERIC=C
```
- In some cases using the debug mode makes Gazebo more reliable, see 
[link](http://answers.gazebosim.org/question/5115/on-startup-of-gazebo-i-get-intermittent-error/).
To start Gazebo in debug mode, run 
`roslaunch baxter_data_acquisition simulation.launch debug:=true` in a SDK 
shell.
