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
    const string default_name = "image_blob";

    RpcServer rpcPort;
    //RpcClient sqPort;
    
    BufferedPort<ImageOf<PixelRgb>> requestPort; // if incoming is normal image
    //BufferedPort<ImageOf<PixelFloat>> requestPort; // if incoming is depth
    //BufferedPort<ImageOf<PixelRgb>> responsePort; // if incoming is normal image
    BufferedPort<Bottle> responsePort; // coords
    BufferedPort<ImageOf<PixelRgb>> blurPort; //  blurred image

    double requestedPeriod = 0.010;
    double areaMin = 370.0;
    double areaMax = 10000.0; // 100000 (100k) was for 640x480, 10000 (10k) for 320x240

    int blur = 0; // type of a blur
    int blur_iter = 31; // max iterations (that's fully blurred)

    //SimpleBlobDetector detector;
    Ptr<SimpleBlobDetector> detector;

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

        // default
        //low = redLow;
        //high = redHigh;
        //color = "red";


        blurPort.open("/"+name+"/blur");

        /*
        // Setup SimpleBlobDetector parameters.
        SimpleBlobDetector::Params params;

        // Change thresholds
        params.minThreshold = 10;
        params.maxThreshold = 200;

        // Filter by Area.
        params.filterByArea = true;
        params.minArea = 1500;

        // Filter by Circularity
        params.filterByCircularity = true;
        params.minCircularity = 0.1;

        // Filter by Convexity
        params.filterByConvexity = true;
        params.minConvexity = 0.87;

        // Filter by Inertia
        params.filterByInertia = true;
        params.minInertiaRatio = 0.01;

        //Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // detector->detect( im, keypoints);
        */

        SimpleBlobDetector::Params params;
        detector = SimpleBlobDetector::create(params);


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

    bool set_blur(int32_t blurtype, int32_t bluri) override {
        blur = blurtype;
        blur_iter = bluri;
        return true;
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

    cv::Mat processing(ImageOf<PixelRgb>* inputImage, Bottle* response) {
        // warning: ‘void* yarp::sig::Image::getIplImage()’ is deprecated: Use yarp::cv::toCvMat instead [-Wdeprecated-declarations]
        //cv::Mat inputMatrix = cv::cvarrToMat((IplImage*) inputImage->getIplImage());
        cv::Mat inputMatrix = yarp::cv::toCvMat(*inputImage);
        cv::Mat img;

        int start = Time::now();

        img = inputMatrix.clone();

        if( blur == 1) {
            //int MAX_KERNEL_LENGTH = 31; // blur_iter
            for ( int i = 1; i < blur_iter; i = i + 2 )
            {
                cv::blur( inputMatrix, img, Size( i, i ), Point(-1,-1) );
            }
        }

        // cv::cvtColor(inputMatrix, inputMatrix, cv::COLOR_BGR2HSV);

        yInfo() << "blur took" << Time::now()-start;
        start = Time::now();

	    std::vector<KeyPoint> keypoints;
	    detector->detect( img, keypoints);
        for (KeyPoint kp : keypoints) {
            Bottle& coords = response->addList();
            coords.addFloat64( kp.pt.x );
            coords.addFloat64( kp.pt.y );
        }

        yInfo() << "blob took" << Time::now()-start;

        return img;

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
            
            Bottle& responseCoords = responsePort.prepare();
            responseCoords.clear();

            yInfo() << "process";

            //ImageOf<PixelRgb> responsecopy = processing(msg0);
            
            // img - cv blurred image
            // response - yarp blurred image  -> blurPort
            // responseCoords - bottle of bottles with x,y coords   -> responsePort

            ImageOf<PixelRgb>& response = blurPort.prepare();
            cv::Mat responsecopy = processing(msg0, &responseCoords);
            
            //ImageOf<PixelRgb> responsecopy = yarp::cv::fromCvMat<yarp::sig::PixelRgb>( img );

            yInfo() << "resize";

            response.resize( 320, 240 );
            int outputWidth = response.width();
            int outputHeight = response.height();


            yInfo() << "copy";

            unsigned char* pOutx = response.getRawImage();
            int outPaddingx = response.getPadding();
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
            blurPort.write();
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
