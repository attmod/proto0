# World frame

https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-forward-kinematics/icub-forward-kinematics/

# Cartesian interface

https://robotology.github.io/robotology-documentation/doc/html/icub_cartesian_interface.html


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

# Timestamps!

http://wiki.icub.org/wiki/Timestamps_in_YARP

Read for checking envelopes:

    yarp read /r1 envelope /port

envelope on normal ports on high frequency can be out of sync https://github.com/robotology/yarp/issues/610, also on bufferedports, but callbacks with buffered ports should be safe?



## Additional fields

Additional fields? You can reimplement the Stamp or the Header - they are supposed to be compatible. Or Portable for more general class.

https://yarp.it/latest//classyarp_1_1os_1_1Header.html

https://www.yarp.it/latest/classyarp_1_1os_1_1Portable.html

See int issue for counters in Stamp/Header https://github.com/robotology/yarp/issues/2365

Writing documentation and example for Portable

https://www.yarp.it/latest/port_expert.html

```cpp
class Target : public Portable {
public:
  int x;
  int y;
  bool write(ConnectionWriter& connection) override
  {
    connection.appendInt32(x);
    connection.appendInt32(y);
    return true;
  }
  bool read(ConnectionReader& connection) override
  {
    x = connection.expectInt32();
    y = connection.expectInt32();
    return !connection.isError();
  }
};
```

and better read/write:

```cpp

bool write(ConnectionWriter& connection) override
{
  connection.appendInt32(BOTTLE_TAG_LIST+BOTTLE_TAG_INT32);
  connection.appendInt32(2); // two elements
  connection.appendInt32(x);
  connection.appendInt32(y);
  connection.convertTextMode(); // if connection is text-mode, convert!
  return true;
}
bool read(ConnectionReader& connection) override
{
  connection.convertTextMode(); // if connection is text-mode, convert!
  int tag = connection.expectInt32();
  if (tag!=BOTTLE_TAG_LIST+BOTTLE_TAG_INT32) return false;
  int ct = connection.expectInt32();
  if (ct!=2) return false;
  x = connection.expectInt32();
  y = connection.expectInt32();
  return !connection.isError();
}
```


# Issue rpc commands quickly

    echo go | yarp rpc /icub-grasp/rpc

# Standard read and write - with xterm

Command for yarpmanager to execute

    yarp read /read /micro_orientation/out

```xml
    <module>
        <name>xterm</name>
        <parameters>-bg black -fg white -T "micro_orientation" -e "yarp read /read /micro_orientation/out"</parameters>
        <node>localhost</node>
        <dependencies>
            <port timeout="20">/micro_orientation/out</port>
        </dependencies>
    </module>
```

and

    yarp write /write/gaze /micro_gaze/in

```xml
<module>
    <name>xterm</name>
    <parameters>-bg black -fg white -T "micro_gaze control" -e "yarp write /write/gaze /micro_gaze/in"</parameters>
    <node>localhost</node>
    <dependencies>
        <port timeout="20">/micro_gaze/in</port>
    </dependencies>
</module>
```


# Time and framerate - official example

https://www.yarp.it/git-master/framerate_2main_8cpp-example.html

# Point clouds

https://www.yarp.it/latest/group__yarp__pointcloud.html

Finding a centroid point:

https://pointclouds.org/documentation/classpcl_1_1_centroid_point.html


# Math operations

https://robotology.github.io/robotology-documentation/doc/html/group__Maths.html

```cpp
double 	iCub::ctrl::dot (const yarp::sig::Matrix &A, int colA, const yarp::sig::Matrix &B, int colB)
 
double 	iCub::ctrl::norm2 (const yarp::sig::Matrix &M, int col)
 
double 	iCub::ctrl::norm (const yarp::sig::Matrix &M, int col)
 
yarp::sig::Vector 	iCub::ctrl::cross (const yarp::sig::Matrix &A, int colA, const yarp::sig::Matrix &B, int colB)
 
yarp::sig::Vector 	iCub::ctrl::Dcross (const yarp::sig::Vector &a, const yarp::sig::Vector &Da, const yarp::sig::Vector &b, const yarp::sig::Vector &Db)
 
yarp::sig::Vector 	iCub::ctrl::Dcross (const yarp::sig::Matrix &A, const yarp::sig::Matrix &DA, int colA, const yarp::sig::Matrix &B, const yarp::sig::Matrix &DB, int colB)
```

# Callbacks on ports

https://www.yarp.it/latest/port_expert.html

```cpp
class DataPort : public BufferedPort<Bottle>
{
    using BufferedPort<Bottle>::onRead;
    void onRead(Bottle& b) override
    {
        // process data in b
    }
};
...
DataPort p;
p.useCallback();  // input should go to onRead() callback
p.open("/in");
```

# Thrift data types

```cpp
    bool: bool

    binary: std::string

    byte: int8_t

    i16: int16_t

    i32: int32_t

    i64: int64_t

    double: double

    string: std::string

    list<t1>: std::vector<t1>

    set<t1>: std::set<t1>

    map<t1,t2>: std::map<T1, T2>
```


# yarp image copy

Since .copy() function is somewhat broken, this is a copy image-cv::Mat-image

```cpp
    void imageCopyRgb( ImageOf<PixelRgb>* org, ImageOf<PixelRgb>* target ) {
        cv::Mat org2 = yarp::cv::toCvMat(*org);

        int outputWidth = org->width();
        int outputHeight = org->height();
        target->resize( outputWidth, outputHeight );

        unsigned char* pOutx = target->getRawImage();
        int outPaddingx = target->getPadding();
        IplImage tempIplx = cvIplImage(org2);
        //IplImage tempIplx = org->getIplImage();
        char* pMatrixx     = tempIplx.imageData;
        int matrixPaddingx = tempIplx.widthStep - tempIplx.width * 3;
        for (int r = 0; r < outputHeight; r++) {
            for(int c = 0 ; c < outputWidth; c++) {             
                *pOutx++ = *pMatrixx++;
                *pOutx++ = *pMatrixx++;
                *pOutx++ = *pMatrixx++;
            }
            pOutx     += outPaddingx;
            pMatrixx  += matrixPaddingx;
        }
    }
```
