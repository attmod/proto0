# Architecture Prototype Zero - solidified

Overview
- Pure gazebo, yarp, C++ modules
- No additional tools both optional and required for building outside of docker and docker-compose

- this version is stacked into a single .cpp file

# Running

Makefile for building and running includes `make build` and `make run`, or both as

    make

Closing the terminator terminal window closes everything.

Currently nvidia gpu support is included in docker-compose file, turn off when on intel/amd machine. Required by some modules (stereo-vision).

system1 can be build and launched by executing

    ./go

which will cmake/make/install the system1 project and launch `yarpmanager`.

Start everything in yarpmanager in

    01_iCub_Gazebo
    02_iCub_system1-solid

both nodes and connections

Currently the preview of the perception stack is being shown and pressing any key on the window progresses the cycle by one.


# License

If not specified otherwise, GPLv2.

