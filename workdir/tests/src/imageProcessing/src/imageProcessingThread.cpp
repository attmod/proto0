// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: Francesco Rea@iit.it
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
 * @file imageProcessingThread.cpp
 * @brief Implementation of the eventDriven thread (see imageProcessingThread.h).
 */

#include <iCub/imageProcessingThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
//using namespace cv;

imageProcessingThread::imageProcessingThread():inputCbPort() {
    robot = "icub";        
}

imageProcessingThread::imageProcessingThread(string _robot, string _configFile):inputCbPort(){
    robot = _robot;
    configFile = _configFile;
}

imageProcessingThread::~imageProcessingThread() {
    // do nothing
}

bool imageProcessingThread::threadInit() {
    idxHue = 0;     
    idxSat = 1;     
    idxBri = 2;     

    valueSat = 1;   
    valueBri = -1;  
    valueHue = -1;  

    deltaSat = 0;
    deltaHue = 0;
    deltaBri = 0;
    
    /* open ports */ 
    //inputCbPort.hasNewImage = false;
    //inputCbPort.useCallback();          // to enable the port listening to events via callback

    if (!inputCbPort.open(getName("/in").c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort.open(getName("/out").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
}

void imageProcessingThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string imageProcessingThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void imageProcessingThread::setInputPortName(string InpPort) {
    this->inputPortName = InpPort;
}

void imageProcessingThread::run() {    
    while (isStopping() != true) {
        if ((inputCbPort.getInputCount()) && (outputPort.getOutputCount())) {
            timeStart = Time::now();
            inputImage = inputCbPort.read(true);
            Stamp timestamp;
            inputCbPort.getEnvelope(timestamp);
            timeEnd = Time::now();
            double time = 1 / (timeEnd-timeStart);
            //printf("time interval %f fps\n", time);
            
            inputHeight = inputImage->height();
            inputWidth  = inputImage->width();
            widthRatio  = floor(inputWidth / outputWidth); 
            heightRatio = floor(inputHeight / outputHeight);
      
            //outputPort.prepare() = *inputImage;
            outputImage = &outputPort.prepare();
            processing();
            outputPort.setEnvelope(timestamp);
            outputPort.write();  
            
        }
    }               
}

/**
 * @param outputMatrix matrix representing the input image and the output of the processing
 * @param tempMatrix matrix passed only as working matrix 
 */
void imageProcessingThread::adjustHSV(cv::Mat& outputMatrix, cv::Mat tempMatrix) {
                double inizio = Time::now();
    //yDebug("converting to HSV height:%d width: %d",outputHeight,outputWidth);


    //convert the image to HSV and adjusting the Hue, Saturation and Value param
    cv::cvtColor(outputMatrix,tempMatrix,CV_BGR2HSV);
                double fine = Time::now(); 
                yDebug("cvtColor1 %f \n", fine-inizio);
                inizio = fine;   
    
    for (int r = 0; r < outputHeight; r ++) {
        for(int c = 0 ; c < outputWidth; c++) { 
            if(tempMatrix.at<cv::Vec3b>(r,c)[idxSat]+deltaSat > 255){
                tempMatrix.at<cv::Vec3b>(r,c)[idxSat] = 255;
            }
            else if(tempMatrix.at<cv::Vec3b>(r,c)[idxSat]+deltaSat < 0) {
                tempMatrix.at<cv::Vec3b>(r,c)[idxSat] = 0;
            }
            else {
                tempMatrix.at<cv::Vec3b>(r,c)[idxSat] += deltaSat;
            }
        }
    }
    
    cv::cvtColor(tempMatrix, outputMatrix, CV_HSV2BGR);                
}

void imageProcessingThread::processing() {
    double inizio = Time::now();
    outputImage->resize(outputWidth, outputHeight);
    unsigned char* pOut = outputImage->getRawImage();
    int outPadding = outputImage->getPadding();
    cv::Mat inputMatrix = cv::cvarrToMat((IplImage*) inputImage->getIplImage());
    cv::Mat tempMatrix = cv::cvarrToMat((IplImage*) outputImage->getIplImage());
    cv::Mat outputMatrix = cv::cvarrToMat((IplImage*) outputImage->getIplImage());


    double fine = Time::now(); 
    // yDebug("processing init %f \n", fine-inizio);
    inizio = fine;  
                
    // ----------------- downsample OpencV ----------------------------------
    //resize image with specific interpolation strategy
    //interpolation – interpolation method:
    //INTER_NN - a nearest-neighbor interpolation
    //INTER_LINEAR - a bilinear interpolation (used by default)
    //INTER_AREA - resampling using pixel area relation. It may be a preferred method for image decimation, 
    //INTER_LANCZOS4 - a Lanczos interpolation over 8x8 pixel neighborhood
    cv::resize(inputMatrix,outputMatrix,outputMatrix.size(), 0, 0, CV_INTER_NN);

    ////adjusting the desired paramters 
    //if (deltaSat != 0) 
    //    adjustHSV(outputMatrix, tempMatrix);

    
    cv::Mat img_gray;
    cv::cvtColor(inputMatrix, img_gray, cv::COLOR_BGR2GRAY);
    cv::Mat img_blur;
    cv::GaussianBlur(img_gray, img_blur, cv::Size(3,3), 0);

    cv::Mat sobelx, sobely, sobelxy;
    cv::Sobel(img_blur, sobelx, CV_64F, 1, 0, 5);
    cv::Sobel(img_blur, sobely, CV_64F, 0, 1, 5);
    cv::Sobel(img_blur, sobelxy, CV_64F, 1, 1, 5);

    // alternative: canny edge detection
    // cv::Mat edges;
    // cv::Canny(img_blur, edges, 100, 200, 3, false);


    //---- preparing the IPL image for the final copy ---------------

    IplImage tempIpl = cvIplImage(sobelx);

    //IplImage tempIpl = cvIplImage(outputMatrix);
    char* pMatrix     = tempIpl.imageData;
    int matrixPadding = tempIpl.widthStep - tempIpl.width * 3;
    //making a copy of it
    for (int r = 0; r < outputHeight; r++) {
        for(int c = 0 ; c < outputWidth; c++) {             
            *pOut++ = *pMatrix++;
            *pOut++ = *pMatrix++;
            *pOut++ = *pMatrix++;
        }
        pOut     += outPadding;
        pMatrix  += matrixPadding;
    }
}

void imageProcessingThread::threadRelease() {
    // nothing     
    inputCallbackPort.interrupt();
    outputPort.interrupt();

    inputCallbackPort.close();
    outputPort.close();
}

void imageProcessingThread::onStop() {
    //inputCallbackPort.interrupt();
    //outputPort.interrupt();

    //inputCallbackPort.close();
    //outputPort.close();
}

