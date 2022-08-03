# About

Color detection - opencv inRange

Ports:

    /image_colors/out - ImageRgb with detected color (in white on black bg, but rgb)
    /image_colors/in - Input image

Start module with:

    --color red   - choose color: red blue green yellow purple
    --name        - starts under a specified name   (STILL TESTING)

# Usage

To check if the proper color is selected:

    echo get_color | yarp rpc /image_colors/rpc



# Authors

Adam Lukomski, S4HRI, IIT, adam.lukomski@iit.it

# License

Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
                   Istituto Italiano di Tecnologia, Genova
Licence: GPLv2 (please see LICENSE file)
