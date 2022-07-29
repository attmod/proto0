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
    BufferedPort<Bottle> responsePort; // if request is vector/float64

    double requestedPeriod = 0.300;
    double areaMin = 370.0;
    double areaMax = 10000.0; // 100000 (100k) was for 640x480, 10000 (10k) for 320x240

    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        // const string name = rf.check("name", 
        //                    Value("/imageProcessing"), 
        //                    "module name (string)").asString();
        const string name = "micro_orientation";

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


    double getOrientation(const vector<Point> &pts)
    {
        //Construct a buffer used by the pca analysis
        int sz = static_cast<int>(pts.size());
        Mat data_pts = Mat(sz, 2, CV_64F);
        for (int i = 0; i < data_pts.rows; i++)
        {
            data_pts.at<double>(i, 0) = pts[i].x;
            data_pts.at<double>(i, 1) = pts[i].y;
        }
        //Perform PCA analysis
        PCA pca_analysis(data_pts, Mat(), PCA::DATA_AS_ROW);
        //Store the center of the object
        Point cntr = Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                        static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
        //Store the eigenvalues and eigenvectors
        vector<Point2d> eigen_vecs(2);
        vector<double> eigen_val(2);
        for (int i = 0; i < 2; i++)
        {
            eigen_vecs[i] = Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                    pca_analysis.eigenvectors.at<double>(i, 1));
            eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
        }
        double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
        return angle;
    }

    void processing(ImageOf<PixelRgb>* inputImage, Bottle* bottle) {
        // warning: ‘void* yarp::sig::Image::getIplImage()’ is deprecated: Use yarp::cv::toCvMat instead [-Wdeprecated-declarations]
        //cv::Mat inputMatrix = cv::cvarrToMat((IplImage*) inputImage->getIplImage());
        cv::Mat inputMatrix = yarp::cv::toCvMat(*inputImage);
        cv::Mat img_gray;
        // gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        cv::cvtColor(inputMatrix, img_gray, cv::COLOR_BGR2GRAY);

        // _, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)
        cv::Mat img_bw;
        cv::threshold( img_gray, img_bw, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU );

        // # Find all the contours in the thresholded image
        vector<vector<Point>> contours;
        cv::findContours( img_bw, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        // contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)
        
        // angles = list()
        
        Vector angles;
        for( size_t k = 0; k < contours.size(); k++ ) {
        // for i, c in enumerate(contours):
            
        //     # Calculate the area of each contour
        //     area = cv.contourArea(c)
            double area = cv::contourArea( contours[k] );
            
        //     # Ignore contours that are too small or too large
        //     if area < 370 or 100000 < area: # was 3700
        //         continue
            if( (area < areaMin) || (area > areaMax) ) {
                continue;
            }

            bottle->addFloat64( getOrientation(contours[k]) );
        }
            
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
            yInfo() << "message incoming";
            
            Bottle& response = responsePort.prepare();
            response.clear();

            processing(msg0,&response);

            //response.addFloat64(  );
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
