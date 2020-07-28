Software of experimental device for study how the light spectrum influences on crop growing rate. It is ROS1 package that using ros-melodic. It was tested on Raspbian buster and Ubuntu 18.04. 

============================

How to install:

Install ROS-melodic on your machine (Linux PC or RPi) by following this tutorial:

http://wiki.ros.org/melodic/Installation/Ubuntu

or if you on Raspberry

http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi

and create catkin workspace by following this tutorial:

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

then if you on Ubintu PC:


```
cd ~/catkin_ws/src
git clone https://github.com/houseofbigseals/farmer_project.git
cd ..
catkin_make --pkg ros_farmer_pc
```

if you on Raspberry Pi

```
cd ~/ros_catkin_ws/src
git clone https://github.com/houseofbigseals/farmer_project.git
cd ..
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/melodic --pkg=ros_farmer_pc
```


=================================

how to use:

hah
