# World frame

https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-forward-kinematics/icub-forward-kinematics/


# Gaze documentation

https://robotology.github.io/robotology-documentation/doc/html/icub_gaze_interface.html

# How to change from vector to bottle?

https://github.com/robotology/yarp/issues/96

```cpp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <stdio.h>

using namespace yarp::os;
using namespace yarp::sig;

int main() {
  Vector v(3);
  v[0] = 1.2;
  v[1] = 2.3;
  v[2] = 4.4;
  Bottle b;
  b.addString("hello");
  b.addList().read(v);
  printf("bottle is %s\n", b.toString().c_str());
  Vector v2;
  b.get(1).asList()->write(v2);
  printf("Vector is %s\n", v2.toString().c_str());
  return 0;
}
```

# Issue rpc commands quickly

    echo go | yarp rpc /icub-grasp/rpc










# yarpmanager

Additional useful entries for yarpmanager are in yarpmanager.xml file.

# Loose advice

Best way to work with modules that come with _random_ pieces of documentation:
- compile them using standard mkdir build && cd build && cmake ../ && make
- don't install yet
- check share directory for modules files and templates

