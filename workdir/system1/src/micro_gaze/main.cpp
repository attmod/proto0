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

    PolyDriver clientGazeCtrl;

    RpcServer rpcPort;
    //RpcClient sqPort;
    
    BufferedPort<Bottle> requestPort; // in: vector xyz where to fixate
    BufferedPort<Bottle> responsePort; // out: vector xyz where we are fixated

    double requestedPeriod = 0.010;

    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        // const string name = rf.check("name", 
        //                    Value("/imageProcessing"), 
        //                    "module name (string)").asString();
        const string name = "micro_gaze";

        Property option;
        option.put("device","gazecontrollerclient");
        option.put("remote","/iKinGazeCtrl");
        option.put("local","/"+name+"/gaze");
        
        if (!clientGazeCtrl.open(option)) {
            yError() << "Can't open gaze controller";
            return false;
        }

        requestPort.open("/"+name+"/in");
        responsePort.open("/"+name+"/out");
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

    /**************************************************************************/
    double getPeriod() override {
        return requestedPeriod;
    }

    /**************************************************************************/
    bool updateModule() override {
        IGazeControl *igaze=NULL;
        if (clientGazeCtrl.isValid()) {
            clientGazeCtrl.view(igaze);
        }

        // don't wait, just snatch if available
        if( auto* msg0 = requestPort.read(false) ) {
            yInfo() << "Bottle received looks like" << msg0->toString().c_str();
            Vector fp(3);
            fp[0]=msg0->get(0).asFloat64();
            fp[1]=msg0->get(1).asFloat64();
            fp[2]=msg0->get(2).asFloat64();
            igaze->lookAtFixationPoint(fp);
        }

        Vector x;
        igaze->getFixationPoint(x);

        Bottle& responseFix = responsePort.prepare(); // get an empty bottle for BufferedPort
        responseFix.clear();
        responseFix.addList().read(x);
        // check
        // cout << "Bottle for sending looks like" << responseFix.toString().c_str();
        responsePort.write();

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
