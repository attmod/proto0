# About

Just the viewer part for point clouds

# Usage

    yarp connect /icubSim/cam/left/rgbImage:o /micro_pointcloud_viewer/rgb:i
    yarp connect /micro_segment/out  /micro_pointcloud_viewer/in

    echo start | yarp rpc /micro_pointcloud_viewer/rpc

You have to use rpc to refresh because VTK blocks the thread for itself and requires main thread, so only RPC for now gets through

# Authors

Adam Lukomski, S4HRI, IIT, adam.lukomski@iit.it

# License

Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
                   Istituto Italiano di Tecnologia, Genova
Licence: GPLv2 (please see LICENSE file)
