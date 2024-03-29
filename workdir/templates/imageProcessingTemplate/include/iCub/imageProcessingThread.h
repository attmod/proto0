// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email: shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file imageProcessingThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _IMAGE_PROCESSING_THREAD_H_
#define _IMAGE_PROCESSING_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp> 

class inputCBPort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > {

public:
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    bool hasNewImage;

    inputCBPort() {
        inputImage = new  yarp::sig::ImageOf<yarp::sig::PixelRgb>; 
    }

    ~inputCBPort() {
        delete inputImage;
    }

    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb>& inputImage1) {
        //printf("Callback for the new image received! \n");
        inputImage = &inputImage1;
        hasNewImage = true;             // to be set to false once done with the events
    }
};


class imageProcessingThread : public yarp::os::Thread {
private:
    int inputWidth;                // width of the input image
    int inputHeight;               // height of the input image
    int outputWidth;               // width of the output image
    int outputHeight;              // height of the output image
    int deltaSat, gainSat;
    int deltaHue, gainHue;
    int deltaBri, gainBri;
    int idxHue, valueHue;     
    int idxSat, valueSat;     
    int idxBri, valueBri;     

    short widthRatio;              // ratio between the input and output image
    short heightRatio;             // ratio between the input and output image
    double timeEnd;                //
    double timeStart;              //
    double verticalOffset;

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImagex;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImagey;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImagexy;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImagec;

    inputCBPort inputCbPort;  // buffered port listening to images through callback
  
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPortx;     // output port to plot event
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPorty;     // output port to plot event
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPortxy;     // output port to plot event
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPortc;     // output port to plot event
    std::string name;                                                                // rootname of all the ports opened by this thread
    
public:
    /**
    * constructor default
    */
    imageProcessingThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    imageProcessingThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~imageProcessingThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /**
    *  on stopping of the thread
    */
    void onStop();

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /**
     * function that sets the dimension of the output image
     */
    void setOutputDimension(int width, int height) {
        outputWidth  = width;
        outputHeight = height;
    }

    /**
     * function that perfoms downsampling (if necessary)
     */
    void processing();

    /**
     * function necessary to adjust Hue Saturation and Brightness in the image
     */
    void adjustHSV(cv::Mat& outputMatrix, cv::Mat tempMatrix);

    void setHuePush(int value) { deltaHue = value; gainHue = 1; };
    void setBriPush(int value) { deltaBri = value; gainBri = 1; };
    void setSatPush(int value) { deltaSat = value; gainSat = 1; };

    void setVerticalOffset(double value) { verticalOffset = value;};
};

#endif  //_IMAGE_PROCESSING_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

