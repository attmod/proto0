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

#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp> 

#include <yarp/cv/Cv.h>

using namespace cv;

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/******************************************************************************/
class ProcessingModule : public RFModule, public rpc_IDL {

    const string default_name = "micro_actions";

    RpcServer rpcPort;
    //RpcClient sqPort;
    
    BufferedPort<Bottle> requestPort; // if incoming is normal image
    //BufferedPort<ImageOf<PixelFloat>> requestPort; // if incoming is depth
    //BufferedPort<ImageOf<PixelRgb>> responsePort; // if incoming is normal image
    BufferedPort<Bottle> responsePort; // if request is vector/float64

    BufferedPort<Bottle> graspPort;
    BufferedPort<Bottle> pointPort;
    BufferedPort<Bottle> reachPort;

    std::string action;

    double requestedPeriod = 0.050;

    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        const string name = rf.check("name", 
                           Value(default_name), 
                           "module name (string)").asString();

        requestPort.open("/"+name+"/in");
        responsePort.open("/"+name+"/out");
        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

        graspPort.open("/"+name+"/grasp:o");
        pointPort.open("/"+name+"/point:o");
        reachPort.open("/"+name+"/reach:o");

        action = rf.check("action", 
                           Value("point"), 
                           "module name (string)").asString();

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

    string get_action() override {
        return action;
    }

    bool grasp() override {
        action = "grasp";
        return true;
    }

    bool point() override {
        action = "point";
        return true;
    }

    bool reach() override {
        action = "reach";
        return true;
    }

    /**************************************************************************/
    double getPeriod() override {
        return requestedPeriod;
    }

    /**************************************************************************/
    bool updateModule() override {
        // don't wait, just snatch if available
        if( Bottle* msg0 = requestPort.read(false) ) {
            //yInfo() << "message incoming";
            std::string s = msg0->get(0).asString();
            if( s == "grasp" )
                grasp();
            if( s == "reach" )
                reach();
            if( s == "point" )
                point();
        }
        Bottle& response = responsePort.prepare();
        response.clear();
        response.addString(action);
        responsePort.write();

        Bottle& responseG = graspPort.prepare();
        responseG.clear();
        if( action == "grasp" ) {
            responseG.addInt32(1);
        } else {
            responseG.addInt32(0);
        }
        graspPort.write();

        Bottle& responseP = pointPort.prepare();
        responseP.clear();
        if( action == "point" ) {
            responseP.addInt32(1);
        } else {
            responseP.addInt32(0);
        }
        pointPort.write();

        Bottle& responseR = reachPort.prepare();
        responseR.clear();
        if( action == "reach" ) {
            responseR.addInt32(1);
        } else {
            responseR.addInt32(0);
        }
        reachPort.write();

        return true;
    }

    bool interruptModule() override {
        // interrupt blocking read
        requestPort.interrupt();
        responsePort.interrupt();
        return true;
    }

    /**************************************************************************/
    bool close() override {
        rpcPort.close();
        requestPort.close();
        responsePort.close();

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
