#!/bin/bash

echo "user id check"
id
echo "hostname check"
hostname


# building the files in case of changing directory structure

echo "delete build folder"
cd /workdir/icub-gazebo && rm -rf build
echo "build"
cd /workdir/icub-gazebo && mkdir build && cd build && cmake ../ && make
cd /workdir/icub-gazebo/build && make install
cd /workdir

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/usr/local/share/iCub/robots:/usr/local/share/gazebo/models:/usr/share/gazebo-11/models/:/usr/local/share
export GAZEBO_MASTER_URI=""
export GAZEBO_MODEL_DATABASE_URI=""


yarpserver --write &


##  push this to background later ...
# yarpmanager-console --application /workdir/app/icub-gazebo.xml --run --connect --exit &


# interactive yarpmanager for testing
cd /workdir
yarpmanager &

##  ... and use a call like this to launch the system
# yarp wait /icub-grasp/rpc
# echo "go" | yarp rpc /icub-grasp/rpc


terminator



