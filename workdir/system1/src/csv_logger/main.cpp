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

#include <chrono>
#include <iostream>
#include <time.h>

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

using namespace std::chrono;


/******************************************************************************/
class ProcessingModule : public RFModule, public rpc_IDL {
    RpcServer rpcPort;
    //RpcClient sqPort;
    
    BufferedPort<Bottle> requestPort;
    BufferedPort<Bottle> responsePort;

    double requestedPeriod = 1.000;

    std::ofstream csvfile;
    std::string filename;

    class CsvPort : public BufferedPort<Bottle>
    {
        public:
            ProcessingModule *m;

        private:
            using BufferedPort<Bottle>::onRead;
            void onRead(Bottle& b) override
            {
                m->csvwrite(b.get(0).asString());
            }
    };

    CsvPort csvPort;


    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        const string default_name = "csv_logger";
        const string name = rf.check("name", 
                           Value(default_name), 
                           "module name (string)").asString();

        //requestPort.open("/"+name+"/in");
        responsePort.open("/"+name+"/out");
        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

        csvPort.m = this;
        csvPort.useCallback();
        csvPort.open("/"+name+"/in");

        //auto localtime = zoned_time{current_zone(), system_clock::now()};
        // auto time = std::time(nullptr);
        // std::string localtime;
        // localtime << std::put_time(std::localtime(&time), "%F %T%z");

        time_t rawtime;
        struct tm * timeinfo;

        time ( &rawtime );
        timeinfo = localtime ( &rawtime );
        //std::string localtime = asctime(timeinfo);

        char localtime[30];
        strftime(localtime, 30, "%Y%m%d-%H%M%S", timeinfo);
        

        //std::cout << localtime << '\n'; // preview
        filename = "example_"+std::string(localtime)+".csv";
        csvfile.open(filename);

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


    void csvwrite(std::string s) {
        csvfile << s << "\n";
        yInfo() << s;
        Bottle& logstring = responsePort.prepare();
        logstring.clear();
        logstring.addString(s);
        responsePort.write();

    }


    /**************************************************************************/
    double getPeriod() override {
        return requestedPeriod;
    }

    /**************************************************************************/
    bool updateModule() override {
        csvfile.close();
        csvfile.open(filename,std::ios_base::app);


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

        csvfile.close();

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
