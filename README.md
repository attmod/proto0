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
  - code inside - single cmake/make combo for all
- [ ] developing the perception module (c++ separate file)
  - [ ] yarp: logpolar
    - BROKEN, returns an error for array size and crashes when connecting camera image with yarp connect
  - [ ] opencv: template for convolution filter
  - [x] opencv: edge detect
    - imageProcessingEdge: sobel and canny, on 320x240 (SegFault on 640x480)
  - [ ] opencv: image threshold
  - [ ] opencv: contour detect
  - [ ] opencv: template for deep neural net
  - [x] yarp: stereo-vision
    - provides SFM and disparity
  - [ ] yarp: segmentation
  - [ ] yarp: caffe
    - requires: apt-get install -y nvidia-cuda-toolkit
    - BROKEN: cmake returns get_target_property() called with non-existent target "caffe".
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

Currently nvidia gpu support is included in docker-compose file, turn off when on intel/amd machine. Required by some modules (stereo-vision).

system1 is NOT being build automatically in the Dockerfile, after running the container you have to build it manually:

    cd /workdir/system1 && mkdir -p build && cd build
    cmake ../
    make && make install


# Spliced repositories

Why splice instead of recursive? Easier to modify for a single project, easier to compile with a single cmake call.


Moving to yarp 3.7:
- mass replaced addInt with addInt16, addDouble with addFloat64, Vocab with Vocab32, etc

https://github.com/robotology/icub-gazebo-grasping-sandbox into workdir/icub-gazebo
- used for gazebo simulation
- License: BSD-3

https://github.com/robotology/attention attention-2.0 83f5835
- already existing solution for attention 2011-2022
- no main documentation / tutorial found yet
- License: GPLv2

https://github.com/robotology/logpolar master f0a0f8e
- compiles without additional modules? only the bare library?
- License: GPLv2

https://github.com/robotology/himrep
- spliced: caffeCoder (broken so far)

https://github.com/robotology/point-cloud-read

https://github.com/attmod/stereo-vision
- set COMPILE_LIB, USE_SFM, and USE_DISPARITYMODULE all by default to TRUE
- requires: apt-get install nvidia-cuda-toolkit
- requires: nonfree in opencv contrib in 4.2 (ubuntu 22.04 can provide SIFT in normal packages in 4.4+), also WITH_FREETYPE=On and installed apt freetype and haarfbuz
- replaced gcc,g++,cc,c++ with links to g++-8 and gcc-8 instead of -9, otherwise CUDA in OpenCV does not compile
- `cv::cuda::DisparityBilateralFilter` is now in `#include <opencv2/cudastereo.hpp>`
- instead of disp16 there is cvIplImage(disp16m)
- CV_BGR2GRAY change to cv::COLOR_BGRA2GRAY

https://github.com/robotology/segmentation
- spliced modules: blobExtractor dispBlobber edison graphBased lbpExtract lumaChroma seg2cloud
- License: GPLv2

https://github.com/robotology/visual-tracking-control
- reaching spliced into tests/, requires superimposemesh
- corrected glew to GLEW
- BROKEN with cv::cvarrToMat

https://github.com/robotology/visuomotor-learning


https://github.com/attmod/tool-incorporation
- for show3D, BROKEN

https://github.com/robotology/superimpose-mesh-lib master da269af
- compiles

# License

If not specified otherwise, GPLv2.

