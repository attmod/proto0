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
#include <sstream>


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
    const string default_name = "image_forward";

    RpcServer rpcPort;
    //RpcClient sqPort;
    
    BufferedPort<ImageOf<PixelRgb>> incameraPort; // if incoming is normal image
    BufferedPort<ImageOf<PixelFloat>> indepthPort; // if incoming is depth
    BufferedPort<ImageOf<PixelRgb>> outcameraPort; // if incoming is normal image
    BufferedPort<ImageOf<PixelFloat>> outdepthPort; // if incoming is depth
    //BufferedPort<ImageOf<PixelRgb>> responsePort; // if incoming is normal image
    //BufferedPort<Bottle> responsePort; // coords
    //BufferedPort<ImageOf<PixelRgb>> responsePort; //  blurred image

    BufferedPort<Bottle> logPort; // if incoming is depth

    double requestedPeriod = 0.010;

    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        const string name = rf.check("name", 
                           Value(default_name), 
                           "module name (string)").asString();

        incameraPort.open("/"+name+"/rgb:i");
        outcameraPort.open("/"+name+"/rgb:o");
        indepthPort.open("/"+name+"/depth:i");
        outdepthPort.open("/"+name+"/depth:o");

        logPort.open("/"+name+"/log:o");

        // requestPort.open("/"+name+"/in");
        // responsePort.open("/"+name+"/out");
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

    void imageCopyFloat( ImageOf<PixelFloat>* org, ImageOf<PixelFloat>* target ) {
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


    /**************************************************************************/
    bool updateModule() override {
        // don't wait, just snatch if available

        if( incameraPort.getPendingReads() > 0 && indepthPort.getPendingReads() > 0 ) {
            if( ImageOf<PixelRgb>* img0 = incameraPort.read(false) ) {
                if( ImageOf<PixelFloat>* depth0 = indepthPort.read(false) ) {

                    yarp::os::Stamp stamp_img0;
                    incameraPort.getEnvelope(stamp_img0);
                    yarp::os::Stamp stamp_depth0;
                    incameraPort.getEnvelope(stamp_depth0);

                    yarp::os::Stamp stamp_img1 = stamp_img0;
                    yarp::os::Stamp stamp_depth1 = stamp_depth0;
                    stamp_img1.update();
                    stamp_depth1.update();

                    //std::string logmessage;

                    stringstream logmessage;
                    logmessage << "image in " << stamp_img0.getCount() << " " << stamp_img0.getTime()
                               << " image out " <<  stamp_img1.getCount() << " " << stamp_img1.getTime()
                               << "depth in " << stamp_depth0.getCount() << " " << stamp_depth0.getTime()
                               << " depth out " << stamp_depth1.getCount() << " " << stamp_depth1.getTime();

                    yInfo()    << "image in " << stamp_img0.getCount() << " " << stamp_img0.getTime()
                               << " image out " <<  stamp_img1.getCount() << " " << stamp_img1.getTime()
                               << "depth in " << stamp_depth0.getCount() << " " << stamp_depth0.getTime()
                               << " depth out " << stamp_depth1.getCount() << " " << stamp_depth1.getTime();

                    Bottle& logstring = logPort.prepare();
                    logstring.clear();
                    logstring.addString(logmessage.str());

                    ImageOf<PixelRgb>& img1 = outcameraPort.prepare();
                    ImageOf<PixelFloat>& depth1 = outdepthPort.prepare();
 
                    img1.copy(*img0);
                    depth1.copy(*depth0);

                    //imageCopyRgb( img0, &img1 );
                    //imageCopyFloat( depth0, &depth1 );

                    outcameraPort.write();
                    outdepthPort.write();
                    logPort.write();
                }
            }
        }

            // //yInfo() << "message incoming";
            
            // responseCoords.clear();

            // yInfo() << "process";

            // //ImageOf<PixelRgb> responsecopy = processing(msg0);
            
            // // img - cv blurred image
            // // response - yarp blurred image  -> blurPort
            // // responseCoords - bottle of bottles with x,y coords   -> responsePort

            // cv::Mat responsecopy = processing(msg0, &responseCoords);
            
            // //ImageOf<PixelRgb> responsecopy = yarp::cv::fromCvMat<yarp::sig::PixelRgb>( img );

            // yInfo() << "resize";

            // response.resize( 320, 240 );
            // int outputWidth = response.width();
            // int outputHeight = response.height();


            // yInfo() << "copy";

            // unsigned char* pOutx = response.getRawImage();
            // int outPaddingx = response.getPadding();
            // IplImage tempIplx = cvIplImage(responsecopy);
            // char* pMatrixx     = tempIplx.imageData;
            // int matrixPaddingx = tempIplx.widthStep - tempIplx.width * 3;
            // for (int r = 0; r < outputHeight; r++) {
            //     for(int c = 0 ; c < outputWidth; c++) {             
            //         *pOutx++ = *pMatrixx++;
            //         *pOutx++ = *pMatrixx++;
            //         *pOutx++ = *pMatrixx++;
            //     }
            //     pOutx     += outPaddingx;
            //     pMatrixx  += matrixPaddingx;
            // }



            // //yInfo() << response.width() << response.height() ;

            // //response->copy( responsecopy );
            // //response.addFloat64(  );

            // yInfo() << "write";

            // responsePort.write();
            // blurPort.write();
        // }

        return true;
    }

    bool interruptModule() override {
        // interrupt blocking read
        // requestPort.interrupt();
        // responsePort.interrupt();
        return true;
    }

    /**************************************************************************/
    bool close() override {
        // rpcPort.close();
        // requestPort.close();
        // responsePort.close();

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
