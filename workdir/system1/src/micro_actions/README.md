# About

Action selection - simple latch mechanism

Input the name of the desired action through rpc or port, the app will store it and redistribute to anyone interested

Ports:

    /micro_actions/in - input the action name
    /micro_actions/out - string output

Bottle with true/false value:

    /micro_actions/grasp:o
    /micro_actions/point:o
    /micro_actions/reach:o

Default period is 0.050 ms

# Usage

Start as

    micro_actions

RPC interface:

    echo grasp | yarp rpc /micro_actions/rpc
    echo reach | yarp rpc /micro_actions/rpc
    echo point | yarp rpc /micro_actions/rpc

    echo get_action | yarp rpc /micro_actions/rpc

Also

    yarp read /r1 /micro_actions/grasp
    yarp read /r2 /micro_actions/point
    yarp read /r3 /micro_actions/reach

    yarp read /r4 /micro_actions/out

    yarp write /w1 /micro_actions/in

# Authors

Adam Lukomski, S4HRI, IIT, adam.lukomski@iit.it

# License

Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
                   Istituto Italiano di Tecnologia, Genova
Licence: GPLv2 (please see LICENSE file)
