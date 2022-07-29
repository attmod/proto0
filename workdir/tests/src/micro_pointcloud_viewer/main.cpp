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
#include "viewer.h"
#include "cardinal_points_grasp.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp> 

#include <thread>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using namespace viewer;
using namespace cardinal_points_grasp;

/******************************************************************************/
class ProcessingModule : public RFModule, public rpc_IDL {

    PolyDriver gaze;

    RpcServer rpcPort;
    //RpcClient sqPort;
    
    //BufferedPort<Bottle> requestPort; // in: vector xyz where to fixate
    BufferedPort<ImageOf<PixelRgb>> rgbPort;
    // BufferedPort<ImageOf<PixelFloat>> depthPort;

    BufferedPort<yarp::sig::PointCloud<DataXYZRGBA>> requestPort; // out: vector xyz where we are fixated
    BufferedPort<Bottle> tablePort; // out: vector xyz where we are fixated


    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_scene{nullptr};
    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_table{nullptr};
    shared_ptr<yarp::sig::PointCloud<DataXYZRGBA>> pc_object{nullptr};
    //yarp::sig::PointCloud<DataXYZRGBA> pc_object;

    Matrix Teye;
    double table_height{numeric_limits<double>::quiet_NaN()};

    double requestedPeriod = 0.250;

    unique_ptr<Viewer> viewer;


    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        // const string name = rf.check("name", 
        //                    Value("/imageProcessing"), 
        //                    "module name (string)").asString();
        const string name = "micro_pointcloud_viewer";

        Property option;
        option.put("device","gazecontrollerclient");
        option.put("remote","/iKinGazeCtrl");
        option.put("local","/"+name+"/gaze");
        
        // if (!clientGazeCtrl.open(option)) {
        //     yError() << "Can't open gaze controller";
        //     return false;
        // }

        if (!gaze.open(option)) {
            yError() << "Unable to open gaze driver!";
            return false;
        }


        requestPort.open("/"+name+"/in");
        // responsePort.open("/"+name+"/out");
        tablePort.open("/"+name+"/table:i");

        rgbPort.open("/"+name+"/rgb:i");
        // depthPort.open("/"+name+"/depth:i");
        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);


        table_height = -0.14;

        pc_object = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();


        viewer = make_unique<Viewer>(10, 370, 350, 350);
        yInfo() << "before start";
        //std::thread thread1(viewer->start);
        //std::thread t1(&Viewer::start, *viewer);
        viewer->start();
        yInfo() << "after start"; /// never reaches this

        // std::thread t1(&Viewer::start_th2, this); // vtk is not thread-safe, sorry


        return true;
    }

    bool start() override {
        updateModule();
        return true;
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

        // viewer->process_events();

        yInfo() << "up";        
        // segment(); // loads images, spits out pc_object

        // IGazeControl *igaze=NULL;
        // if (clientGazeCtrl.isValid()) {
        //     clientGazeCtrl.view(igaze);
        // }

        // don't wait, just snatch if available
        if( yarp::sig::PointCloud<DataXYZRGBA>* msg0 = requestPort.read(false) ) {
            yInfo() << "Bottle received";

            if( Bottle* h = tablePort.read(false) ) {
                table_height = h->get(0).asFloat64();

                yInfo() << "Height set";
            }

            yInfo() << "copy";
            pc_object->copy( *msg0 );
            //pc_object.copy( *msg0 );
            //msg0->copy( *pc_object );

            yInfo() << "segment";
            segment();

            // Vector fp(3);
            // fp[0]=msg0->get(0).asFloat64();
            // fp[1]=msg0->get(1).asFloat64();
            // fp[2]=msg0->get(2).asFloat64();
            // igaze->lookAtFixationPoint(fp);
        }

        // Vector x;
        // igaze->getFixationPoint(x);

        // yarp::sig::PointCloud<DataXYZRGBA>& responseFix = responsePort.prepare(); // get an empty bottle for BufferedPort
        // responseFix.clear();
        // responseFix.copy( *pc_object );
        //responseFix.addList().read(x);
        // // check
        // // cout << "Bottle for sending looks like" << responseFix.toString().c_str();
        // responsePort.write();

        return true;
    }

    bool interruptModule() override {
        viewer->stop();
        // interrupt blocking read
        // requestPort.interrupt();
        // responsePort.interrupt();
        return true;
    }

    bool segment() {
        // // get image data
        auto* rgbImage = rgbPort.read();
        // auto* depthImage = depthPort.read();

        // if ((rgbImage == nullptr) || (depthImage == nullptr)) {
        //     yError() << "Unable to receive image data!";
        //     return false;
        // }

        // if ((rgbImage->width() != depthImage->width()) ||
        //     (rgbImage->height() != depthImage->height()) ) {
        //     yError() << "Received image data with wrong size!";
        //     return false;
        // }

        yInfo() << "image size";
        const auto w = rgbImage->width();
        const auto h = rgbImage->height();

        // get camera extrinsics
        yInfo() << "camera";
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

        // // aggregate image data in the point cloud of the whole scene
        // pc_scene = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        // Vector x{0., 0., 0., 1.};
        // for (int v = 0; v < h; v++) {
        //     for (int u = 0; u < w; u++) {
        //         const auto rgb = (*rgbImage)(u, v);
        //         const auto depth = (*depthImage)(u, v);
                
        //         if (depth > 0.F) {
        //             x[0] = depth * (u - .5 * (w - 1)) / fov_h;
        //             x[1] = depth * (v - .5 * (h - 1)) / fov_h;
        //             x[2] = depth;
        //             x = Teye * x;
                
        //             pc_scene->push_back(DataXYZRGBA());
        //             auto& p = (*pc_scene)(pc_scene->size() - 1);
        //             p.x = (float)x[0];
        //             p.y = (float)x[1];
        //             p.z = (float)x[2];
        //             p.r = rgb.r;
        //             p.g = rgb.g;
        //             p.b = rgb.b;
        //             p.a = 255;
        //         }
        //     }
        // }

        //savePCL("/workspace/pc_scene.off", pc_scene);

        // segment out the table and the object
        // pc_table = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        // pc_object = make_shared<yarp::sig::PointCloud<DataXYZRGBA>>();
        // table_height = RANSAC(pc_scene, pc_table, pc_object);
        // if (isnan(table_height)) {
        //     yError() << "Segmentation failed!";
        //     return false;
        // }

        //savePCL("/workspace/pc_table.off", pc_table);
        //savePCL("/workspace/pc_object.off", pc_object);

        // update viewer
        yInfo() << "viewer";
        Vector cam_foc;
        igaze->get3DPointOnPlane(0, {w/2., h/2.}, {0., 0., 1., -table_height}, cam_foc);
        viewer->addTable({cam_foc[0], cam_foc[1], cam_foc[2]}, {0., 0., 1.});
        viewer->addObject(pc_object);
        viewer->addCamera({cam_x[0], cam_x[1], cam_x[2]}, {cam_foc[0], cam_foc[1], cam_foc[2]},
                           {0., 0., 1.}, view_angle);

        // if (pc_object->size() > 0) {
        //     yInfo() << "Found something";
        //     return true;
        // } else {
        //     yInfo() << "Unable to segment any object!";
        //     return false;
        // }
        return true;
    }

    /**************************************************************************/
    bool close() override {
        rpcPort.close();
        // requestPort.close();
        // responsePort.close();

        return true;
    }

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
