# About

Orientation of shapes in an image

https://docs.opencv.org/4.x/d1/dee/tutorial_introduction_to_pca.html

Ports:

    /micro_orientation/in - Image
    /micro_orientation/out - Bottle with Float64 - one double per each detected shape

# Usage

    yarp connect /icubSim/cam/left/rgbImage:o /micro_orientation/in

    yarp read /read /micro_orientation/out



# Authors

Adam Lukomski, S4HRI, IIT, adam.lukomski@iit.it

# License

Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
                   Istituto Italiano di Tecnologia, Genova
Licence: GPLv2 (please see LICENSE file)
