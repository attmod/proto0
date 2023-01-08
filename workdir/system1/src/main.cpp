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
#include <yarp/pcl/Pcl.h> // compatibility with PCL functions

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

#include "cardinal_points_grasp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


// display the name for the image type
// cout << type2str( mask.type() ); // 8UC1 is PixelMono
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


// Shows multiple (up to 12) images in a single window
//     ShowManyImages("Image", 6, img1, img2, img3, img4, img5, img6);
//https://github.com/opencv/opencv/wiki/DisplayManyImages
void ShowManyImages(string title, int nArgs, ...) {
    int size;
    int i;
    int m, n;
    int x, y;
    int w, h;
    float scale;
    int max;

    if(nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if(nArgs > 14) {
        printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
        return;
    }
    else if (nArgs == 1) {
        w = h = 1;
        size = 300;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 300;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 300;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }

    cv::Mat DispImage = cv::Mat::zeros(cv::Size(100 + size*w, 60 + size*h), CV_8UC3);

    va_list args;
    va_start(args, nArgs);

    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
        cv::Mat img = va_arg(args, cv::Mat);

        if(img.empty()) {
            printf("Invalid arguments");
            return;
        }

        x = img.cols;
        y = img.rows;

        max = (x > y)? x: y;
        scale = (float) ( (float) max / size );

        if( i % w == 0 && m!= 20) {
            m = 20;
            n+= 20 + size;
        }

        cv::Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
        cv::Mat temp; cv::resize(img,temp, cv::Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }

    cv::namedWindow( title, 1 );
    cv::imshow( title, DispImage);
    cv::waitKey();

    va_end(args);
}



class ProcessingAction {
    // blob
    cv::Ptr<cv::SimpleBlobDetector> detector;
    cv::SimpleBlobDetector::Params params;

    // gaze
    IGazeControl *igaze;
    PolyDriver gaze;

    // grasp, reach
    PolyDriver arm_r,arm_l;
    PolyDriver hand_r,hand_l;
    //ICartesianControl *icart_r, *icart_l;
    vector<int> fingers = {7, 8, 9, 10, 11, 12, 13, 14, 15};

    public:
        // segment
        shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_scene{nullptr};
        shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_table{nullptr};
        shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_object{nullptr};
        Matrix Teye;
        double table_height{numeric_limits<double>::quiet_NaN()};

        bool helperArmDriverOpener(PolyDriver& arm, const Property& options) {
            const auto t0 = Time::now(); // TODO change auto to actual type
            while (Time::now() - t0 < 10.) {
                // this might fail if controller is not connected to solver yet
                if (arm.open(const_cast<Property&>(options))) {
                    return true;
                }
                Time::delay(1.);
            }
            return false;
        }



        ProcessingAction() {
            // blob
            cv::SimpleBlobDetector::Params params;
            params.filterByArea = true;
            params.minArea = 3;
            detector = cv::SimpleBlobDetector::create(params);

            // gaze
            std::string name = "experiment";
            Property option;
            option.put("device","gazecontrollerclient");
            option.put("remote","/iKinGazeCtrl");
            option.put("local","/"+name+"/gaze");
            
            if (!gaze.open(option)) {
                yError() << "Can't open gaze controller";
            }

            if (gaze.isValid()) {
                gaze.view(igaze);
            }


            Property arm_r_options;
            arm_r_options.put("device", "cartesiancontrollerclient");
            arm_r_options.put("local", "/"+name+"/arm_r");
            arm_r_options.put("remote", "/icubSim/cartesianController/right_arm");
            helperArmDriverOpener(arm_r, arm_r_options);

            Property arm_l_options;
            arm_l_options.put("device", "cartesiancontrollerclient");
            arm_l_options.put("local", "/"+name+"/arm_l");
            arm_l_options.put("remote", "/icubSim/cartesianController/left_arm");
            helperArmDriverOpener(arm_l, arm_l_options);

            Property hand_r_options;
            hand_r_options.put("device", "remote_controlboard");
            hand_r_options.put("local", "/"+name+"/hand_r");
            hand_r_options.put("remote", "/icubSim/right_arm");
            hand_r.open(hand_r_options);

            Property hand_l_options;
            hand_l_options.put("device", "remote_controlboard");
            hand_l_options.put("local", "/"+name+"/hand_l");
            hand_l_options.put("remote", "/icubSim/left_arm");
            hand_l.open(hand_l_options);


            // set up velocity of arms' movements
            {
                vector<PolyDriver*> polys({&arm_r, &arm_l});
                for (auto poly:polys) {
                    ICartesianControl* iarm;
                    poly->view(iarm);
                    iarm->setTrajTime(.6);
                }
            }

            // enable position control of the fingers
            {
                vector<PolyDriver*> polys({&hand_r, &hand_l});
                for (auto poly:polys) {
                    IControlMode* imod;
                    poly->view(imod);
                    imod->setControlModes(fingers.size(), fingers.data(), vector<int>(fingers.size(), VOCAB_CM_POSITION).data());
                }
            }
        }

        cv::Mat image_to_cv_mat(ImageOf<PixelRgb> *image) {
            return yarp::cv::toCvMat(*image);
        }

        cv::Mat image_to_cv_mat(ImageOf<PixelFloat> *image) {
            return yarp::cv::toCvMat(*image);
        }

        // can be used on respose in BufferedPorts
        void cv_mat_to_image( cv::Mat org2, ImageOf<PixelRgb>* target ) {
            int outputWidth = org2.size().width;
            int outputHeight = org2.size().height;
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

        void copy_image( ImageOf<PixelRgb>* org, ImageOf<PixelRgb>* target ) {
            cv::Mat org2 = yarp::cv::toCvMat(*org);
            cv_mat_to_image( org2, target );
        }

        /*   ------------------------------------  */
        /*                                         */
        /*              blur, blob                 */
        /*                                         */
        /*   ------------------------------------  */


        cv::Mat blur(cv::Mat input, int blur_iter=5) {
            cv::Mat img = input.clone();

            //int MAX_KERNEL_LENGTH = 31; // blur_iter
            for ( int i = 1; i < blur_iter; i = i + 2 )
            {
                cv::blur( input, img, cv::Size( i, i ), cv::Point(-1,-1) );
                int threshold_value = 3;
                int threshold_type = 0;
                int const max_binary_value = 255;
                cv::cvtColor( img, img, cv::COLOR_RGB2GRAY );
                cv::threshold( img, img, threshold_value, max_binary_value, threshold_type );
                cv::cvtColor( img, img, cv::COLOR_GRAY2RGB );
            }

            // cv::cvtColor(inputMatrix, inputMatrix, cv::COLOR_BGR2HSV);
            return img;
        }

        std::vector<cv::KeyPoint> blob(cv::Mat img) {
            std::vector<cv::KeyPoint> keypoints;
            detector->detect( img, keypoints);

            return keypoints;
        }

        yarp::os::Bottle blob_bottle(cv::Mat img) {
            std::vector<cv::KeyPoint> keypoints = blob(img);

            yarp::os::Bottle response;
            for (cv::KeyPoint kp : keypoints) {
                yarp::os::Bottle& coords = response.addList();
                coords.addFloat64( kp.pt.x );
                coords.addFloat64( kp.pt.y );
            }
            return response;
        }

        /*   ------------------------------------  */
        /*                                         */
        /*                color                    */
        /*                                         */
        /*   ------------------------------------  */


        cv::Mat color(cv::Mat input, std::string color="red") {
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

            if( !strcmp( color.c_str(), "red" ) ) {
                low = redLow;
                high = redHigh;
            } else if ( !strcmp( color.c_str(), "blue" ) ) {
                low = blueLow;
                high = blueHigh;
            } else if ( !strcmp( color.c_str(), "green" ) ) {
                low = greenLow;
                high = greenHigh;
            } else if ( !strcmp( color.c_str(), "yellow" ) ) {
                low = yellowLow;
                high = yellowHigh;
            } else if ( !strcmp( color.c_str(), "purple" ) ) {
                low = purpleLow;
                high = purpleHigh;
            } else if ( !strcmp( color.c_str(), "orange" ) ) {
                low = orangeLow;
                high = orangeHigh;
            }

            cv::Mat mask;

            cv::cvtColor(input, input, cv::COLOR_BGR2HSV);
            cv::inRange(input, low, high, mask);
            cv::cvtColor(mask, mask, cv::COLOR_GRAY2RGB);

            return mask;
        }

        /*   ------------------------------------  */
        /*                                         */
        /*                   edges                 */
        /*                                         */
        /*   ------------------------------------  */


        cv::Mat edge(cv::Mat input, std::string method="x") {
            cv::Mat img_gray;
            cv::cvtColor(input, img_gray, cv::COLOR_BGR2GRAY);

            cv::Mat img_blur;
            cv::GaussianBlur(img_gray, img_blur, cv::Size(3,3), 0);

            cv::Mat out;
            if( method == "x" )
                cv::Sobel(img_blur, out, CV_64F, 1, 0, 5);
            if( method == "y" )
                cv::Sobel(img_blur, out, CV_64F, 0, 1, 5);
            if( method == "y" )
                cv::Sobel(img_blur, out, CV_64F, 1, 1, 5);
            if( method == "canny" )
                cv::Canny(img_blur, out, 100, 200, 3, false);
            cv::convertScaleAbs(out, out);
            //convertScaleAbs(grad_y, abs_grad_y);
            cv::cvtColor(out, out, cv::COLOR_GRAY2RGB);
            return out;
        }

        /*   ------------------------------------  */
        /*                                         */
        /*              orientation                */
        /*                                         */
        /*   ------------------------------------  */
        

        double get_orientation(const vector<cv::Point> &pts) {
            //Construct a buffer used by the pca analysis
            int sz = static_cast<int>(pts.size());
            cv::Mat data_pts = cv::Mat(sz, 2, CV_64F);
            for (int i = 0; i < data_pts.rows; i++)
            {
                data_pts.at<double>(i, 0) = pts[i].x;
                data_pts.at<double>(i, 1) = pts[i].y;
            }
            //Perform PCA analysis
            cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
            //Store the center of the object
            cv::Point cntr = cv::Point(static_cast<int>(pca_analysis.mean.at<double>(0, 0)),
                            static_cast<int>(pca_analysis.mean.at<double>(0, 1)));
            //Store the eigenvalues and eigenvectors
            vector<cv::Point2d> eigen_vecs(2);
            vector<double> eigen_val(2);
            for (int i = 0; i < 2; i++)
            {
                eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
                                        pca_analysis.eigenvectors.at<double>(i, 1));
                eigen_val[i] = pca_analysis.eigenvalues.at<double>(i);
            }
            double angle = atan2(eigen_vecs[0].y, eigen_vecs[0].x); // orientation in radians
            return angle;
        }

        yarp::os::Bottle orientation(cv::Mat inputMatrix) {
            double areaMin = 370.0;
            double areaMax = 10000.0; // 100000 (100k) was for 640x480, 10000 (10k) for 320x240
            cv::Mat img_gray;
            cv::cvtColor(inputMatrix, img_gray, cv::COLOR_BGR2GRAY);

            cv::Mat img_bw;
            cv::threshold( img_gray, img_bw, 50, 255, cv::THRESH_BINARY | cv::THRESH_OTSU );

            vector<vector<cv::Point>> contours;
            cv::findContours( img_bw, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
            
            Vector angles;
            yarp::os::Bottle bottle;
            for( size_t k = 0; k < contours.size(); k++ ) {
                double area = cv::contourArea( contours[k] );
                if( (area < areaMin) || (area > areaMax) ) {
                    continue;
                }

                bottle.addFloat64( get_orientation(contours[k]) );
            }
            return bottle;
        }

        /*   ------------------------------------  */
        /*                                         */
        /*              segmentation               */
        /*                                         */
        /*   ------------------------------------  */

        double RANSAC(std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_scene,
                    std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_table,
                    std::shared_ptr<yarp::sig::PointCloud<yarp::sig::DataXYZRGBA>> pc_object,
                    const int num_points = 100) {

            // generate random indexes
            std::random_device rnd_device;
            std::mt19937 mersenne_engine(rnd_device());
            std::uniform_int_distribution<int> dist(0, pc_scene->size() - 1);
            auto gen = std::bind(dist, mersenne_engine);
            std::vector<int> remap(num_points);
            std::generate(std::begin(remap), std::end(remap), gen);

            // implement RANSAC
            const auto threshold_1 = .01F; // [cm]
            for (size_t i = 0; i < remap.size(); i++) {
                auto& pi = (*pc_scene)(remap[i]);
                auto h = 0.F;
                size_t n = 0;
                for (size_t j = 0; j < remap.size(); j++) {
                    const auto& pj = (*pc_scene)(remap[j]);
                    if (std::fabs(pj.z - pi.z) < threshold_1) {
                        h += pj.z;
                        n++;
                    }
                }
                h /= n;

                if (n > (remap.size() >> 1)) {
                    pc_table->clear();
                    pc_object->clear();
                    const auto threshold_2 = h + threshold_1;
                    for (size_t i = 0; i < pc_scene->size(); i++) {
                        const auto& p = (*pc_scene)(i);
                        if (p.z < threshold_2) {
                            pc_table->push_back(p);
                        } else {
                            pc_object->push_back(p);
                        }
                    }
                    return h;
                }
            }

            return std::numeric_limits<double>::quiet_NaN();
        }

        // waaay too many output args - just check invoke as
        // if( segment(...) {
        //     processing.pc_table
        //     processing.pc_object
        // }
        //
        bool segment(ImageOf<PixelRgb>* rgbImage, ImageOf<PixelFloat>* depthImage) {
            const auto w = rgbImage->width();
            const auto h = rgbImage->height();

            // get camera extrinsics
            IGazeControl* igaze;
            gaze.view(igaze);
            Vector cam_x, cam_o;
            igaze->getLeftEyePose(cam_x, cam_o);
            Teye = axis2dcm(cam_o);
            Teye.setSubcol(cam_x, 0, 3);

            // get camera intrinsics
            Bottle info;
            igaze->getInfo(info);
            const auto fov_h = info.find("camera_intrinsics_left").asList()->get(0).asFloat64();
            const auto view_angle = 2. * std::atan((w / 2.) / fov_h) * (180. / M_PI);

            // aggregate image data in the point cloud of the whole scene
            pc_scene = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
            Vector x{0., 0., 0., 1.};
            for (int v = 0; v < h; v++) {
                for (int u = 0; u < w; u++) {
                    const auto rgb = (*rgbImage)(u, v);
                    const auto depth = (*depthImage)(u, v);
                    
                    if (depth > 0.F) {
                        x[0] = depth * (u - .5 * (w - 1)) / fov_h;
                        x[1] = depth * (v - .5 * (h - 1)) / fov_h;
                        x[2] = depth;
                        x = Teye * x;
                    
                        pc_scene->push_back(DataXYZRGBA());
                        auto& p = (*pc_scene)(pc_scene->size() - 1);
                        p.x = (float)x[0];
                        p.y = (float)x[1];
                        p.z = (float)x[2];
                        p.r = rgb.r;
                        p.g = rgb.g;
                        p.b = rgb.b;
                        p.a = 255;
                    }
                }
            }

            //savePCL("/workspace/pc_scene.off", pc_scene);

            // segment out the table and the object
            pc_table = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
            pc_object = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
            table_height = RANSAC(pc_scene, pc_table, pc_object);
            if (isnan(table_height)) {
                yError() << "Segmentation failed!";
                return false;
            }

            //savePCL("/workspace/pc_table.off", pc_table);
            //savePCL("/workspace/pc_object.off", pc_object);

            // update viewer
            // Vector cam_foc;
            // igaze->get3DPointOnPlane(0, {w/2., h/2.}, {0., 0., 1., -table_height}, cam_foc);
            // viewer->addTable({cam_foc[0], cam_foc[1], cam_foc[2]}, {0., 0., 1.});
            // viewer->addObject(pc_object);
            // viewer->addCamera({cam_x[0], cam_x[1], cam_x[2]}, {cam_foc[0], cam_foc[1], cam_foc[2]},
            //                   {0., 0., 1.}, view_angle);

            if (pc_object->size() > 0) {
                yInfo() << "Found something";
                return true;
            } else {
                yInfo() << "Unable to segment any object!";
                return false;
            }
        }

        /*   ------------------------------------  */
        /*                                         */
        /*                 PCL                     */
        /*                                         */
        /*   ------------------------------------  */

        // PCL -> YARP
        yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> pcl_to_yarp(pcl::PointCloud<pcl::PointXYZRGBA> cloud) {
            yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloud;
            yarp::pcl::fromPCL<pcl::PointXYZRGBA, yarp::sig::DataXYZRGBA>(cloud, yarpCloud);
            return yarpCloud;
        }   

        // YARP -> PCL
        pcl::PointCloud<pcl::PointXYZRGBA> yarp_to_pcl( yarp::sig::PointCloud<yarp::sig::DataXYZRGBA> yarpCloud ) {
            pcl::PointCloud<pcl::PointXYZRGBA> cloud2;
            yarp::pcl::toPCL<yarp::sig::DataXYZRGBA, pcl::PointXYZRGBA>(yarpCloud, cloud2);
            return cloud2;
        }

        // void viewerUpdate(pcl::visualization::PCLVisualizer& viewer)
        // {
        //     viewer.setBackgroundColor (1.0, 0.5, 1.0);
        //     pcl::PointXYZ o;
        //     o.x = 1.0;
        //     o.y = 0;
        //     o.z = 0;
        //     viewer.addSphere (o, 0.25, "sphere", 0);
        //     std::cout << "i only run once" << std::endl;
        // }

        void show_cloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) {
            //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

            pcl::visualization::CloudViewer viewer("Cloud Viewer");
    
            //blocks until the cloud is actually rendered
            viewer.showCloud(cloud);
    
            // //use the following functions to get access to the underlying more advanced/powerful
            // //PCLVisualizer
            
            // //This will only get called once
            // viewer.runOnVisualizationThreadOnce (viewerOneOff);
            
            // //This will get called once per visualization iteration
            // viewer.runOnVisualizationThread (viewerPsycho);
            while (!viewer.wasStopped ())
            {
            }
        }

        /*   ------------------------------------  */
        /*                                         */
        /*                 gaze                    */
        /*                                         */
        /*   ------------------------------------  */



        // look at the pixel in the camera image plane, with depth z
        void look_pixel(double x, double y, double z=1.0) {
            int camSel=0;   // select the image plane: 0 (left), 1 (right)

            Vector px(2);   // specify the pixel where to look
            px[0]=x;
            px[1]=y;

            Vector fixation_point;
            igaze->get3DPoint(camSel,px,z,fixation_point);
            igaze->lookAtFixationPoint(fixation_point);
        }

        void look_fixation( double x, double y, double z ) {
            Vector fp(3);
            fp[0]=x;
            fp[1]=y;
            fp[2]=z;
            igaze->lookAtFixationPoint(fp);
        }

        void look_fixation( Vector fp ) {
            igaze->lookAtFixationPoint(fp);
        }


        // std::vector<double> get_fixation() {
        //     std::vector<double> out;
        //     Vector x;
        //     igaze->getFixationPoint(x);
        //     out.push_back( x[0] );
        //     out.push_back( x[1] );
        //     out.push_back( x[2] );
        //     return out;
        // }

        Vector get_fixation() {
            Vector x;
            igaze->getFixationPoint(x);
            return x;
        }

        /*   ------------------------------------  */
        /*                                         */
        /*                 cartesian               */
        /*                                         */
        /*   ------------------------------------  */

        bool home(){
            // home gazing
            //IGazeControl* igaze;
            //gaze.view(igaze);
            igaze->lookAtAbsAnglesSync({0., -50., 10.});

            // home arms
            {
                Vector x{-.25, .3, .1};
                vector<PolyDriver*> polys({&arm_r, &arm_l});
                ICartesianControl* iarm;
                for (auto poly:polys) {                
                    poly->view(iarm);
                    iarm->goToPositionSync(x);                
                    x[1] = -x[1];
                }
                // wait only for the last arm
                // iarm->waitMotionDone();
            }

            igaze->waitMotionDone();
            return true;
        }


        bool reach(double x, double y, double z) {
                Vector reach_x{x,y,z};
                vector<PolyDriver*> polys({&arm_r});
                ICartesianControl* iarm;
                for (auto poly:polys) {                
                    poly->view(iarm);
                    iarm->goToPositionSync(reach_x);                
                }
                // wait only for the last arm
                //iarm->waitMotionDone();
                iarm->waitMotionDone(.1, 3.);
                return true;
        }


        bool grasp(double x=-0.45, double y=0.0, double z=0.0, double angle=160.0, double gx=0.05, double gy=0.02, double gz=0.08) {
            // bottle init from a string
            //Bottle sqParams("-0.435105 -0.0838161 -0.0573888 166.035 0.0313233 0.0220981 0.0783015");
            Bottle sqParams;
            yInfo() << "default sqParams in";
            sqParams.addFloat64(x);
            sqParams.addFloat64(y);
            sqParams.addFloat64(z);
            sqParams.addFloat64(angle);
            sqParams.addFloat64(gx);
            sqParams.addFloat64(gy);
            sqParams.addFloat64(gz);

            const Vector sqCenter{sqParams.get(0).asFloat64(),
                                sqParams.get(1).asFloat64(),
                                sqParams.get(2).asFloat64()};

            // set up the hand pre-grasp configuration
            IControlLimits* ilim;
            hand_r.view(ilim);
            double pinkie_min, pinkie_max;
            ilim->getLimits(15, &pinkie_min, &pinkie_max);
            const vector<double> pregrasp_fingers_posture{60., 80., 0., 0., 0., 0., 0., 0., pinkie_max};

            // keep gazing at the object
            IGazeControl* igaze;
            gaze.view(igaze);
            igaze->setTrackingMode(true);
            igaze->lookAtFixationPoint(sqCenter);

            // apply cardinal points grasp algorithm
            ICartesianControl* iarm;
            arm_r.view(iarm);
            auto grasper_r = make_shared<cardinal_points_grasp::CardinalPointsGrasp>(cardinal_points_grasp::CardinalPointsGrasp("right", pregrasp_fingers_posture));
            const auto candidates_r = grasper_r->getCandidates(sqParams, iarm);

            arm_l.view(iarm);
            auto grasper_l = make_shared<cardinal_points_grasp::CardinalPointsGrasp>(cardinal_points_grasp::CardinalPointsGrasp("left", pregrasp_fingers_posture));
            const auto candidates_l = grasper_l->getCandidates(sqParams, iarm);

            // aggregate right and left arms candidates
            auto candidates = candidates_r.first;
            candidates.insert(candidates.end(), candidates_l.first.begin(), candidates_l.first.end());
            std::sort(candidates.begin(), candidates.end(), cardinal_points_grasp::CardinalPointsGrasp::compareCandidates);

            // some safety checks
            if (candidates.empty()) {
                yError() << "No good grasp candidates found!";
                //lookAtDeveloper();
                //shrug();
                return false;
            }

            // extract relevant info
            const auto& best = candidates[0];
            const auto& type = get<0>(best);
            const auto& T = get<2>(best);

    //        viewer->showCandidates(candidates);

            // select arm corresponing to the best candidate
            shared_ptr<cardinal_points_grasp::CardinalPointsGrasp> grasper;
            IPositionControl* ihand;
            int context;
            if (type == "right") {
                grasper = grasper_r;
                hand_r.view(ihand);
                arm_r.view(iarm);
                context = candidates_r.second;
            } else {
                grasper = grasper_l;
                hand_l.view(ihand);
                arm_l.view(iarm);
                context = candidates_l.second;
            }

            // target pose that allows grasping the object
            const auto point = T.getCol(3).subVector(0, 2); // was x
            const auto o = dcm2axis(T);

            // enable the context used by the algorithm
            iarm->stopControl();
            iarm->restoreContext(context);
            iarm->setInTargetTol(.001);
            iarm->setTrajTime(1.);

            yInfo() << "put the hand in the pre-grasp configuration";

            // put the hand in the pre-grasp configuration
            // ihand->setRefAccelerations(fingers.size(), fingers.data(), vector<double>(fingers.size(), numeric_limits<double>::infinity()).data());
            // ihand->setRefSpeeds(fingers.size(), fingers.data(), vector<double>{60., 60., 60., 60., 60., 60., 60., 60., 200.}.data());
            // ihand->positionMove(fingers.size(), fingers.data(), pregrasp_fingers_posture.data());
            // auto done = false;
            // while (!done) {
            //     Time::delay(1.);
            //     ihand->checkMotionDone(fingers.size(), fingers.data(), &done);
            // };

            //yInfo() << "reach for the pre-grasp pose....";

            // reach for the pre-grasp pose
            //const auto dir = point - sqCenter;
            //iarm->goToPoseSync(point + .05 * dir / norm(dir), o);
            //iarm->waitMotionDone(.1, 3.);
            
            yInfo() << "reach for the object";

            // reach for the object
            iarm->goToPoseSync(point, o);
            //iarm->waitMotionDone(.1, 3.);


            ihand->setRefAccelerations(fingers.size(), fingers.data(), vector<double>(fingers.size(), numeric_limits<double>::infinity()).data());
            ihand->setRefSpeeds(fingers.size(), fingers.data(), vector<double>{60., 60., 60., 60., 60., 60., 60., 60., 200.}.data());
            ihand->positionMove(fingers.size(), fingers.data(), pregrasp_fingers_posture.data());
            auto done = false;
            while (!done) {
                Time::delay(1.);
                ihand->checkMotionDone(fingers.size(), fingers.data(), &done);
            };



            yInfo() << "close fingers";

            // close fingers
            ihand->positionMove(fingers.size(), fingers.data(), vector<double>{60., 80., 40., 35., 40., 35., 40., 35., pinkie_max}.data());


            arm_r.view(iarm);
            iarm->setTrackingMode(false);
            arm_l.view(iarm);
            iarm->setTrackingMode(false);
            igaze->setTrackingMode(false);


            return true;

        }


};



/******************************************************************************/
class MainModule : public RFModule {

    BufferedPort<ImageOf<PixelRgb>> cameraPort; 
    BufferedPort<ImageOf<PixelFloat>> depthPort; 

    ProcessingAction processing;


    bool configure(ResourceFinder& rf) override {
        // const string name = rf.check("name", 
        //                    Value("/imageProcessing"), 
        //                    "module name (string)").asString();
        const string name = "experiment";

        cameraPort.open("/"+name+"/rgb:in");
        depthPort.open("/"+name+"/depth:in");

        processing.look_fixation( -0.4, 0, -0.3 );


        return true;
    }

    // yarp::os::Time uses actually a good perfomance clock, at least in C++ on linux so far
    double time_start() {
        double t0 = yarp::os::Time::now();
        return t0;
    }

    double time_step(double t0, std::string comment) {
        double t1 = yarp::os::Time::now();
        yInfo() << comment << " at " << t1-t0;
        return t1;
    }

    /**************************************************************************/
    bool updateModule() override {
        if( ImageOf<PixelRgb>* msg0 = cameraPort.read(false) ) {
            if( ImageOf<PixelFloat>* msg1 = depthPort.read(false) ) {
                double t0 = time_start();
                cv::Mat original = processing.image_to_cv_mat(msg0);
                time_step(t0, "image convert");
                cv::Mat colored = processing.color( original, "blue" );
                time_step(t0, "image color");
                cv::Mat blurred = processing.blur( colored );
                time_step(t0, "blur");
                cv::Mat edged = processing.edge( original );
                time_step(t0, "edge");
                // double-check types of images to be shown
                yInfo() << type2str(original.type()) << type2str(colored.type()) << type2str(blurred.type()) << type2str(edged.type());
                // ShowManyImages( "images", 4, original, blurred, colored, edged );
                
                if( processing.segment( msg0, msg1 ) ) {
                    pcl::PointCloud<pcl::PointXYZRGBA> cloud = processing.yarp_to_pcl( *processing.pc_object );
                    time_step(t0, "segment");
                    
                    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudptr = cloud.makeShared();
                    //processing.showCloud( cloudptr ); // works, but perspective is awful and is blocking

                    Eigen::Vector4f centroid;
                    pcl::compute3DCentroid(cloud, centroid);
                    yInfo() << "centroid:" << centroid[0] << " " <<  centroid[1] << " " <<   centroid[2] << " " <<   centroid[3] << " \n";
                }
                
                
                time_step(t0, "finishes everything");
                
                ShowManyImages( "images", 4, original, blurred, colored, edged); // blocking call!

            }
        }
        return true;
    }

    bool interruptModule() override {
        // interrupt blocking read
        return true;
    }

    /**************************************************************************/
    bool close() override {
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

    MainModule module;
    return module.runModule(rf);
}
