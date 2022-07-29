# About

Segmentation

Inputs: images (rgb and float)

    /micro_segment/rgb:i
    /micro_segment/depth:i

Output: 

    /micro_segment/out  - yarp::sig::PointCloud<DataXYZRGBA>
    /micro_segment/table:o   - Bottle with Float64


# Usage

Connect images:

    yarp connect /icubSim/cam/left/depthImage:o /micro_segment/depth:i
    yarp connect /icubSim/cam/left/rgbImage:o /micro_segment/rgb:i


Table height:

    yarp read /read /micro_segment/table:o

# Authors

Adam Lukomski, S4HRI, IIT, adam.lukomski@iit.it

# License

Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
                   Istituto Italiano di Tecnologia, Genova
Licence: GPLv2 (please see LICENSE file)
