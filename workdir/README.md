# init.sh

This is run inside a docker container

Functions:
- recompile icub-gazebo (make sure it is up-to-date)
- set paths for robotology, static /usr/local
- start yarpserver

Closing the terminator terminal window ends the session

# go

Running

    ./go
    
will recompile system1, install it, and launch yarpmanager

# system1

initial system with placeholders

Building by hand:

    cd system1
    mkdir -p build && cd build
    cmake ../
    make
    make install

