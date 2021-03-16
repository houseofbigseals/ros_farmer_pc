#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash;

tun_addr=$(ip -f inet addr show tun0 | awk '/inet / {print $2}' | cut -d/ -f1);
export ROS_MASTER_URI=http://$tun_addr:11311/;
export ROS_IP=$tun_addr;
export ROS_HOSTNAME=$tun_addr;

today=$(date +"%d_%m_%Y_%H:%M:%S");
outfile_name="${today}_exp_log.out";
nohup roslaunch ros_farmer_pc current_experiment.launch > $outfile_name 2>&1 &