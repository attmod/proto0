# usage

start on a console

    micro_grasp

then either start a placeholder grasp by issuing:

    echo go | yarp rpc /icub-grasp/rpc

or connect to /icub-grasp/msg:i and send a bottle with 7 Float64 values that describe the positions of the object

    x y z angle gx gy gz

for example:

    -0.435105 -0.0838161 -0.0573888 166.035 0.0313233 0.0220981 0.0783015

Sending a vector of lenght 7 triggers full grasp, sending a vector of length 3 goes for simple reach with right arm.

# 

based on https://github.com/robotology/icub-gazebo-grasping-sandbox
