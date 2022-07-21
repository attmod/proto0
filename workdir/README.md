# init.sh

Functions:
- recompile icub-gazebo (make sure it is up-to-date)
- set paths for robotology, static /usr/local
- start yarpserver and yarpmanager with app/icub-gazebo.xml

Closing the terminator terminal window ends the session

# system1

initial system with placeholders

Building by hand:

    cd system1
    mkdir -p build && cd build
    cmake ../
    make
    make install

