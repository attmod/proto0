# Architecture Prototype Zero

Overview
- Pure gazebo, yarp, C++ modules
- No additional tools both optional and required for building outside of docker and docker-compose
- No experiment, no decision system or higher level logic, no message broker
- No grasping, just inverse kinematics and moving the hand to the proximity of the object - orientation of the hand as the parameter, but simple open/close
- Simple convolutional filters for edge detection, disparity maps

# Checklist

- [x] create github repository
  - https://github.com/attmod/proto0
- [x] creating a "one click solution" for building/running the environment
  - Makefile - make (build+run), make build, make run
  - classic docker-compose (nvidia gpu by default)
  - Consider dockyman for running later, if still using X.org forwarding
- [ ] developing the perception module (c++ separate file)
- [ ] developing action module (c++ separate file)
- [ ] creating actions (reaching, grasping, pointing) and testing on Gazebo
- [ ] defining target functions that validate the actions. They return TRUE once the action is successful
- [ ] defining a set of visual features needed for achieving the actions. Input to the action modules
- [ ] defining a rate for each perception/action module
  - current VGA camera setup tops out at 640x480 with 15fps on the real robot
  - try to include graphs with delays introduced at each stage (Gannt-style)
- [ ] defining cost functions for each perception/action module
- [ ] create simulation for playing the actions and showing the evolution of validate and cost functions 
- [ ] (optional) statistics of cost functions respect to the different actions

# Running

Makefile for building and running includes make build and make run, or both as

    make

Closing the terminator terminal window closes everything.

Currently nvidia gpu support is included in docker-compose file, turn off when on intel/amd machine.

# Spliced repositories

https://github.com/robotology/icub-gazebo-grasping-sandbox into workdir/icub-gazebo
- used for gazebo simulation
- License: BSD-3

https://github.com/robotology/attention
- already existing solution for attention 2011-2022
- no main documentation / tutorial found yet
- License: GPLv2

https://github.com/robotology/himrep

https://github.com/robotology/logpolar

https://github.com/robotology/point-cloud-read

https://github.com/robotology/segmentation

https://github.com/robotology/stereo-vision

https://github.com/robotology/visual-tracking-control

https://github.com/robotology/visuomotor-learning


# License

If not specified otherwise, GPLv2.

