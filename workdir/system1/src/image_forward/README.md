# About

Image forward

Ports:

    /image_forward/rgb:i
    /image_forward/rgb:o
    /image_forward/depth:i
    /image_forward/depth:o
    /image_forward/log:o
    /image_forward/rpc


# Usage

To check if the proper color is selected:

    yarp connect /icubSim/cam/left/rgbImage:o  /image_forward/rgb:i
    yarp connect /icubSim/cam/left/depthImage:o  /image_forward/depth:i



# Authors

Adam Lukomski, S4HRI, IIT, adam.lukomski@iit.it

# License

Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
                   Istituto Italiano di Tecnologia, Genova
Licence: GPLv2 (please see LICENSE file)
