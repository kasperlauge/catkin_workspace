# Getting started with the entire Research and Development project

The documentation of using PX4 with multiple UAVs and Gazebo is very sparse. This section serves as a fulfilling guide of getting started with all of that. This specific repository is part of a Research and Development course at Aarhus University. and will include custom code for this project.

References:

[https://dev.px4.io/v1.9.0/en/setup/dev_env.html](https://dev.px4.io/v1.9.0/en/setup/dev_env.html)

[https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html)

[http://gazebosim.org/](http://gazebosim.org/)

[https://www.ros.org/](https://www.ros.org/)


#### Download toolchain

Ubuntu 18.04 is a requirement.

From the link [https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html](https://dev.px4.io/v1.9.0/en/setup/dev_env_linux.html) follow the guide from the 'Gazebo with ROS Melodic' headline. This mainly involves downloading and running the script: ubuntu_sim_ros_melodic.sh

This part downloads and installs ROS melodic and Gazebo 9 with some important dependencies which is the important part of this script. Furthermore this creates a catkin workspace called catkin_ws. We won't use this, but because of convenience we run this script anyway. The script ends at a Firmware directory. This directory is for the PX4 firmware code which we originally got following this guide: [https://dev.px4.io/v1.9.0/en/setup/building_px4.html](https://dev.px4.io/v1.9.0/en/setup/building_px4.html). We have a fork of the repository which should be used instead because of some extensions we are making. Go to the Firmware directory.

#### Get Firmware

```bash
cd ~/Firmware
git clone https://github.com/kasperlauge/Firmware.git
cd ~/Firmware
```

#### Running simulation environment with extensions
Now we have the Firmware, Gazebo and ROS melodic installed. Now we should run it to see if the environment works as it should. Go to the Firmware repository and start everything. The following commands is how to start the simulation environment with multiple UAVs and a custom world we have built.

```bash
cd ~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd):$(pwd)/Tools/sitl_gazebo
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
make px4_sitl gazebo (Only neccesary the first time, close the Gazebo after running this and execute the next command to start Gazebo with the right parameters)
roslaunch px4 multi_uav_mavros_sitl.launch vehicle:=iris_sensors world:=/home/kasperlauge/Firmware/Tools/sitl_gazebo/worlds/aarslev.world
```

Now the Gazebo simulation should be started, showing one or two UAVs. And it should be equipped with the right sensors within the world called aarslev.

To be able to control the UAVs we can use a tool called QGroundControl which can be downloaded following this: [https://dev.px4.io/v1.9.0/en/qgc/](https://dev.px4.io/v1.9.0/en/qgc/) and from here essentially: [http://qgroundcontrol.com/downloads/](http://qgroundcontrol.com/downloads/). Start the QGroundControl program when downloaded. Now you should be able to fly the UAV around in the world using this tool.

#### Create ROS nodes being able to talk to the simulation environment

To be able to create rosnodes which can aggregate several commands this repository becomes relevant. This repository is a ROS catkin workspace where we through mavros and mavlink can communicate with the UAVs in the simulation environment. To get started with this repository do the following: 

#### Create ROS nodes

Clone this repository in ~/ and set it up

```bash
cd ~/
git clone https://github.com/kasperlauge/catkin_workspace.git
cd ./catkin_workspace
sh ./setup.sh
```

Now you can create catkin packages within this workspace which can communicate with the UAVs. An example of this could be:

```bash
cd ~/catkin_workspace
catkin_create_pkg image_data roscpp std_msgs mavros_msgs sensor_msgs
```

After that the CMakeLists.txt file within the image_data directory should be modified by adding this part

```bash
add_executable(process_image_data src/process_image_data.cpp)
```

Now execute in ~/catkin_workspace

```bash
catkin build
```

This should now be runnable by doing the following

First run the Firmware repository by executing the previously stated commands.
Then execute:

```bash
rosrun image_data process_image_data
```

Do the stuff you would like to do in C++ within that catkin package. Remember to build when you have made changes.


### Visualizing sensor data

Sensor data can be visualized using the tool rviz. After running the multi UAV commands as described above rvis can be started using the following commands:

```bash
rosrun rviz rviz
```

To be able to visualize data for eg. the lidar sensor you will need to first make a mapping from the data, starting the following rosnode like this:

```bash
rosrun tf static_transform_publisher 0 0 0 0 0 0 map iris_sensors_0/lidar_lidar_iris_link 50
```

This command maps the lidar data exposed on the topic iris_sensors_0/lidar_lidar_iris_link to the global 'map'. You might need to restart rviz afterwards. Now you should be able to add a lidar data based on the specific topic in rviz which will now visualize the data from the lidar sensor. The camera sensor can also be visualized by adding the topic to rviz.