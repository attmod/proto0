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
    if (!outputPortx.open(getName("/sobelx").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    if (!outputPorty.open(getName("/sobely").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    if (!outputPortxy.open(getName("/sobelxy").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    if (!outputPortc.open(getName("/canny").c_str())) {
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
        if ((inputCbPort.getInputCount()) && (outputPortx.getOutputCount()) && (outputPorty.getOutputCount()) && (outputPortxy.getOutputCount()) && (outputPortc.getOutputCount())) {
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
            outputImagex = &outputPortx.prepare();
            outputImagey = &outputPorty.prepare();
            outputImagexy = &outputPortxy.prepare();
            outputImagec = &outputPortc.prepare();
            processing();
            outputPortx.setEnvelope(timestamp);
            outputPortx.write();  
            outputPorty.setEnvelope(timestamp);
            outputPorty.write();  
            outputPortxy.setEnvelope(timestamp);
            outputPortxy.write();  
            outputPortc.setEnvelope(timestamp);
            outputPortc.write();  
            
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
    outputImagex->resize(outputWidth, outputHeight);
    outputImagey->resize(outputWidth, outputHeight);
    outputImagexy->resize(outputWidth, outputHeight);
    outputImagec->resize(outputWidth, outputHeight);
    unsigned char* pOutx = outputImagex->getRawImage();
    unsigned char* pOuty = outputImagey->getRawImage();
    unsigned char* pOutxy = outputImagexy->getRawImage();
    unsigned char* pOutc = outputImagec->getRawImage();
    int outPaddingx = outputImagex->getPadding();
    int outPaddingy = outputImagey->getPadding();
    int outPaddingxy = outputImagexy->getPadding();
    int outPaddingc = outputImagec->getPadding();
    cv::Mat inputMatrix = cv::cvarrToMat((IplImage*) inputImage->getIplImage());
//    cv::Mat tempMatrix = cv::cvarrToMat((IplImage*) outputImage->getIplImage());
    cv::Mat outputMatrixx = cv::cvarrToMat((IplImage*) outputImagex->getIplImage());
    cv::Mat outputMatrixy = cv::cvarrToMat((IplImage*) outputImagey->getIplImage());
    cv::Mat outputMatrixxy = cv::cvarrToMat((IplImage*) outputImagexy->getIplImage());
    cv::Mat outputMatrixc = cv::cvarrToMat((IplImage*) outputImagec->getIplImage());


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
//    cv::resize(inputMatrix,outputMatrix,outputMatrix.size(), 0, 0, CV_INTER_NN);

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
    cv::Mat edges;
    cv::Canny(img_blur, edges, 100, 200, 3, false);


    //---- preparing the IPL image for the final copy ---------------

    IplImage tempIplx = cvIplImage(sobelx);

    //IplImage tempIpl = cvIplImage(outputMatrix);
    char* pMatrixx     = tempIplx.imageData;
    int matrixPaddingx = tempIplx.widthStep - tempIplx.width * 3;
    //making a copy of it
    for (int r = 0; r < outputHeight; r++) {
        for(int c = 0 ; c < outputWidth; c++) {             
            *pOutx++ = *pMatrixx++;
            *pOutx++ = *pMatrixx++;
            *pOutx++ = *pMatrixx++;
        }
        pOutx     += outPaddingx;
        pMatrixx  += matrixPaddingx;
    }


    IplImage tempIply = cvIplImage(sobely);

    //IplImage tempIpl = cvIplImage(outputMatrix);
    char* pMatrixy     = tempIply.imageData;
    int matrixPaddingy = tempIply.widthStep - tempIply.width * 3;
    //making a copy of it
    for (int r = 0; r < outputHeight; r++) {
        for(int c = 0 ; c < outputWidth; c++) {             
            *pOuty++ = *pMatrixy++;
            *pOuty++ = *pMatrixy++;
            *pOuty++ = *pMatrixy++;
        }
        pOuty     += outPaddingy;
        pMatrixy  += matrixPaddingy;
    }

    IplImage tempIplxy = cvIplImage(sobelxy);

    //IplImage tempIpl = cvIplImage(outputMatrix);
    char* pMatrixxy     = tempIplxy.imageData;
    int matrixPaddingxy = tempIplxy.widthStep - tempIplxy.width * 3;
    //making a copy of it
    for (int r = 0; r < outputHeight; r++) {
        for(int c = 0 ; c < outputWidth; c++) {             
            *pOutxy++ = *pMatrixxy++;
            *pOutxy++ = *pMatrixxy++;
            *pOutxy++ = *pMatrixxy++;
        }
        pOutxy     += outPaddingxy;
        pMatrixxy  += matrixPaddingxy;
    }


    IplImage tempIplc = cvIplImage(edges);

    //IplImage tempIpl = cvIplImage(outputMatrix);
    char* pMatrixc     = tempIplc.imageData;
    int matrixPaddingc = tempIplc.widthStep - tempIplc.width * 3;
    //making a copy of it
    for (int r = 0; r < outputHeight; r++) {
        for(int c = 0 ; c < outputWidth; c++) {             
            *pOutc++ = *pMatrixc++;
            *pOutc++ = *pMatrixc++;
            *pOutc++ = *pMatrixc++;
        }
        pOutc     += outPaddingc;
        pMatrixc  += matrixPaddingc;
    }


}

void imageProcessingThread::threadRelease() {
    // nothing     
    inputCallbackPort.interrupt();
    outputPortx.interrupt();
    outputPorty.interrupt();
    outputPortxy.interrupt();
    outputPortc.interrupt();

    inputCallbackPort.close();
    outputPortx.close();
    outputPorty.close();
    outputPortxy.close();
    outputPortc.close();
}

void imageProcessingThread::onStop() {
    //inputCallbackPort.interrupt();
    //outputPort.interrupt();

    //inputCallbackPort.close();
    //outputPort.close();
}

