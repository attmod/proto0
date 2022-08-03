// Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
//                    Istituto Italiano di Tecnologia, Genova
// Licence: GPLv2 (please see LICENSE file)

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cmath>
#include <limits>
#include <random>
#include <fstream>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Value.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/PointCloud.h>
#include <yarp/math/Math.h>

#include "rpc_IDL.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp> 

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

/******************************************************************************/
class ProcessingModule : public RFModule, public rpc_IDL {
    RpcServer rpcPort;
    //RpcClient sqPort;
    
    BufferedPort<ImageOf<PixelRgb>> requestPort; // if incoming is normal image
    //BufferedPort<ImageOf<PixelFloat>> requestPort; // if incoming is depth
    BufferedPort<ImageOf<PixelRgb>> responsePort; // if incoming is normal image
    //BufferedPort<Bottle> responsePort; // if request is vector/float64

    BufferedPort<Bottle> fpsPort;
    double requestedPeriod = 0.010;
    double rate_count = 0;
    int rate_fps = 0;
    double rate_last = 0;

    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        // const string name = rf.check("name", 
        //                    Value("/imageProcessing"), 
        //                    "module name (string)").asString();
        const string name = "image-to-image";

        requestPort.open("/"+name+"/in");
        responsePort.open("/"+name+"/out");
        fpsPort.open("/"+name+"/fps:o");
        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

        return true;
    }

    bool start() override {
        return false;
    }

    bool stop() override {
        return false;
    }

    bool running() override {
        return false;
    }

    bool set_rate(double rate) override {
        if( rate != 0.0 ) {
            requestedPeriod = 1.0/rate;
        } else {
            return false;
        }
        return true;
    }

    int get_rate() override {
        return rate_fps;
    }

    /**************************************************************************/

    void update_rate() {
        rate_count++;
        if( rate_last == 0 ) {
            rate_last = Time::now();
        }

        if( rate_count > 100 ) {
            double now = Time::now();
            rate_fps = round( rate_count / ( now - rate_last )); 
            rate_last = now;
            rate_count = 0;
        }

        if( rate_count % 10 == 0 ) {
            Bottle& fps_msg = fpsPort.prepare();
            fps_msg.clear();
            fps_msg.addInt64(rate_fps);
            fpsPort.write();
        }
    }

    double getPeriod() override {
        return requestedPeriod;
    }

    /**************************************************************************/
    bool updateModule() override {
        // don't wait, just snatch if available
        // if( auto* msg0 = requestPort.read(false) ) {
        //     yInfo() << "message incoming";
        // }

        // typical vector response
        // Bottle& responseFix = responsePort.prepare(); // get an empty bottle for BufferedPort
        // responseFix.clear();
        // responseFix.addList().read(x);

        // typical float response
        // responseFix.addFloat64(x);


        update_rate();
        return true;
    }

    bool interruptModule() override {
        // interrupt blocking read
        requestPort.interrupt();
        responsePort.interrupt();
        rpcPort.interrupt();
        fpsPort.interrupt();
        return true;
    }

    /**************************************************************************/
    bool close() override {
        rpcPort.close();
        requestPort.close();
        responsePort.close();
        fpsPort.close();
        return true;
    }
};

/******************************************************************************/
int main(int argc, char *argv[]) {
    Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "Unable to find YARP server!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    ProcessingModule module;
    return module.runModule(rf);
}
