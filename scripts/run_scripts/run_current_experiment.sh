#!/usr/bin/env bash

source /opt/ros/melodic/setup.bash

echo "you started script as $USER  with id $EUID"
tun_addr=$(ip -f inet addr show tun0 | awk '/inet / {print $2}' | cut -d/ -f1)
export ROS_MASTER_URI=http://$tun_addr:11311/
export ROS_IP=$tun_addr
export ROS_HOSTNAME=$tun_addr
echo "we have set IP as $tun_addr"
echo "$(printenv | grep ROS)"

today=$(date +"%d_%m_%Y_%H_%M_%S")
outfile_name="$HOME/${today}_exp_log.out"
nohup roslaunch ros_farmer_pc current_experiment.launch > $outfile_name 2>&1 &
echo "we have set nohup log path as $outfile_name"
PID=$!
echo "ros experiment launched with pid: $PID"