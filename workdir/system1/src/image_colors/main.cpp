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
    RpcServer rpcPort;
    //RpcClient sqPort;
    
    BufferedPort<ImageOf<PixelRgb>> requestPort; // if incoming is normal image
    //BufferedPort<ImageOf<PixelFloat>> requestPort; // if incoming is depth
    //BufferedPort<ImageOf<PixelRgb>> responsePort; // if incoming is normal image
    BufferedPort<ImageOf<PixelRgb>> responsePort; // if request is vector/float64

    double requestedPeriod = 0.010;
    double areaMin = 370.0;
    double areaMax = 10000.0; // 100000 (100k) was for 640x480, 10000 (10k) for 320x240


    // https://pastebin.com/yFYMTFpt
    cv::Scalar yellowLow = cv::Scalar(25, 130, 180);
    cv::Scalar yellowHigh = cv::Scalar(45, 255, 255);
    cv::Scalar greenLow = cv::Scalar(46, 40, 40);
    cv::Scalar greenHigh = cv::Scalar(70, 255, 255);
    cv::Scalar blueLow = cv::Scalar(100, 150, 150);
    cv::Scalar blueHigh = cv::Scalar(140, 255, 255);
    cv::Scalar purpleLow = cv::Scalar(148, 117, 89);
    cv::Scalar purpleHigh = cv::Scalar(152, 255, 255);
    cv::Scalar redLow = cv::Scalar(170, 140, 160);
    cv::Scalar redHigh = cv::Scalar(180, 255, 255);
    cv::Scalar orangeLow = cv::Scalar(2, 200, 200);
    cv::Scalar orangeHigh = cv::Scalar(20, 255, 255);

    cv::Scalar low;
    cv::Scalar high;

    std::string color;

    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        const string default_name = "image_colors";
        const string name = rf.check("name", 
                           Value(default_name), 
                           "module name (string)").asString();

        requestPort.open("/"+name+"/in");
        responsePort.open("/"+name+"/out");
        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

        // default
        //low = redLow;
        //high = redHigh;
        //color = "red";

        color = rf.check("color", 
                           Value("red"), 
                           "module name (string)").asString();
        set_color( color );

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

    bool set_min(double minarea) override {
        if( minarea != 0.0 ) {
            areaMin = minarea;
        } else {
            return false;
        }
        return true;
    }

    bool set_max(double maxarea) override {
        if( maxarea != 0.0 ) {
            areaMax = maxarea;
        } else {
            return false;
        }
        return true;
    }

    bool set_color(string clr) {
        // std::string clr;
        if( !strcmp( clr.c_str(), "red" ) ) {
            low = redLow;
            high = redHigh;
            color = "red";
        } else if ( !strcmp( clr.c_str(), "blue" ) ) {
            low = blueLow;
            high = blueHigh;
            color = "blue";
        } else if ( !strcmp( clr.c_str(), "green" ) ) {
            low = greenLow;
            high = greenHigh;
            color = "green";
        } else if ( !strcmp( clr.c_str(), "yellow" ) ) {
            low = yellowLow;
            high = yellowHigh;
            color = "yellow";
        } else if ( !strcmp( clr.c_str(), "purple" ) ) {
            low = purpleLow;
            high = purpleHigh;
            color = "purple";
        } else if ( !strcmp( clr.c_str(), "orange" ) ) {
            low = orangeLow;
            high = orangeHigh;
            color = "orange";
        } else {
            return false;
        }

        return true;
    }

    bool red() override {
            low = redLow;
            high = redHigh;
            color = "red";
            return true;
    }

    bool orange() override {
            low = orangeLow;
            high = orangeHigh;
            color = "orange";
            return true;
    }

    bool blue() override {
            low = blueLow;
            high = blueHigh;
            color = "blue";
            return true;
    }

    bool green() override {
            low = greenLow;
            high = greenHigh;
            color = "green";
            return true;
    }

    bool yellow() override {
            low = yellowLow;
            high = yellowHigh;
            color = "yellow";
            return true;
    }
    bool purple() override {
            low = purpleLow;
            high = purpleHigh;
            color = "purple";
            return true;
    }

    bool set_color(int32_t newlow1, int32_t newlow2, int32_t newlow3, int32_t newhigh1, int32_t newhigh2, int32_t newhigh3) override {
            low = cv::Scalar(newlow1, newlow2, newlow3);;
            high = cv::Scalar(newhigh1, newhigh2, newhigh3);;
            color = "custom";
            return true;
    }


    string get_color() override {
        return color;
    }

    string type2str(int type) {
        string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch ( depth ) {
            case CV_8U:  r = "8U"; break;
            case CV_8S:  r = "8S"; break;
            case CV_16U: r = "16U"; break;
            case CV_16S: r = "16S"; break;
            case CV_32S: r = "32S"; break;
            case CV_32F: r = "32F"; break;
            case CV_64F: r = "64F"; break;
            default:     r = "User"; break;
        }

        r += "C";
        r += (chans+'0');

        return r;
    }

    cv::Mat processing(ImageOf<PixelRgb>* inputImage) {
        // warning: ‘void* yarp::sig::Image::getIplImage()’ is deprecated: Use yarp::cv::toCvMat instead [-Wdeprecated-declarations]
        //cv::Mat inputMatrix = cv::cvarrToMat((IplImage*) inputImage->getIplImage());
        cv::Mat inputMatrix = yarp::cv::toCvMat(*inputImage);
        cv::Mat img_gray;

        cv::cvtColor(inputMatrix, inputMatrix, cv::COLOR_BGR2HSV);

        yInfo() << "color";

        int start = Time::now();
        cv::Mat mask;
        cv::inRange(inputMatrix, low, high, mask);
        yInfo() << "inRange took" << Time::now()-start;

        yInfo() << type2str( mask.type() ); // 8UC1 is PixelMono
        yInfo() << "ret";

        cv::cvtColor(mask, mask, cv::COLOR_GRAY2RGB);

        //return yarp::cv::fromCvMat<yarp::sig::PixelRgb>(mask);
        return mask;
        
        //return angles;
    }

    /**************************************************************************/
    double getPeriod() override {
        return requestedPeriod;
    }

    /**************************************************************************/
    bool updateModule() override {
        // don't wait, just snatch if available
        if( ImageOf<PixelRgb>* msg0 = requestPort.read(false) ) {
            //yInfo() << "message incoming";
            
            ImageOf<PixelRgb> *response = &responsePort.prepare();
            //response.clear();

            yInfo() << "process";

            //ImageOf<PixelRgb> responsecopy = processing(msg0);
            cv::Mat responsecopy = processing(msg0);

            yInfo() << "resize";

            response->resize( 320, 240 );
            int outputWidth = response->width();
            int outputHeight = response->height();


            yInfo() << "copy";

            unsigned char* pOutx = response->getRawImage();
            int outPaddingx = response->getPadding();
            IplImage tempIplx = cvIplImage(responsecopy);
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



            //yInfo() << response.width() << response.height() ;

            //response->copy( responsecopy );
            //response.addFloat64(  );

            yInfo() << "write";

            responsePort.write();
        }

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
