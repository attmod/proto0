# About

    csv_logger

and connect incoming messages to it

    yarp connect /some-port/log:o /csv_logger/log:i

if you wish to see output line-by-line on a terminal:

    yarp read /read/csv_logger /csv_logger/out

# Usage


# Authors

Adam Lukomski, S4HRI, IIT, adam.lukomski@iit.it

# License

Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
                   Istituto Italiano di Tecnologia, Genova
Licence: GPLv2 (please see LICENSE file)
