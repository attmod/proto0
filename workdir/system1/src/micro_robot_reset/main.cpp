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
class GrasperModule : public RFModule, public rpc_IDL {
    // PolyDriver arm_r,arm_l;
    // PolyDriver hand_r,hand_l;
    // PolyDriver torso;
    // PolyDriver gaze;

    // ICartesianControl *icart_r, *icart_l;

    RpcServer rpcPort;
    // RpcClient sqPort;
    // BufferedPort<Bottle> requestedSuperquadric;

    // vector<int> fingers = {7, 8, 9, 10, 11, 12, 13, 14, 15};
    // Bottle sqParams;

    // Vector reach_x{0.0,0.0,0.0};
    
    // Vector action_point{0.0,0.0,0.0};
    // bool action_point_valid=false;
    // double action_orientation=0;
    // bool action_orientation_valid=false;
    // Vector action_size{0.0,0.0,0.0};
    // bool action_size_valid=false;

    // BufferedPort<Bottle> pointPort;
    // BufferedPort<Bottle> orientationPort;
    // BufferedPort<Bottle> sizePort;

    // BufferedPort<Bottle> graspActionPort;
    // BufferedPort<Bottle> reachActionPort;
    // BufferedPort<Bottle> pointActionPort;


    bool attach(RpcServer& source) override {
        return this->yarp().attachAsServer(source);
    }

    // auto helperArmDriverOpener(PolyDriver& arm, const Property& options) {
    //     const auto t0 = Time::now();
    //     while (Time::now() - t0 < 10.) {
    //         // this might fail if controller is not connected to solver yet
    //         if (arm.open(const_cast<Property&>(options))) {
    //             return true;
    //         }
    //         Time::delay(1.);
    //     }
    //     return false;
    // }


    class BodyPart {
        public:
            PolyDriver driver;
            IPositionControl *pos;
            IEncoders *encs;
            int nj;

            Vector encoders;
            Vector command;
            Vector tmp;
            Vector command_home;
            // open( "/icubSim/right_arm", name )
            void open(std::string part,std::string name) {
                Property options;
                options.put("device", "remote_controlboard");
                options.put("local", "/"+name+part);
                options.put("remote", part);
                driver.open(options);

                driver.view(pos);
                driver.view(encs);

                pos->getAxes(&nj);

                encoders.resize(nj);
                tmp.resize(nj);
                command.resize(nj);

                int i;
                for (i = 0; i < nj; i++) {
                    tmp[i] = 50.0;
                }
                pos->setRefAccelerations(tmp.data());
                for (i = 0; i < nj; i++) {
                    tmp[i] = 10.0;
                    pos->setRefSpeed(i, tmp[i]);
                }

                read_home();

            }

            Vector read() {
                while(!encs->getEncoders(encoders.data()))
                {
                    Time::delay(0.1);
                    yInfo() << "reading encoders...";
                }
                yInfo() << "... done reading encoders";
                yInfo() << encoders.toString();
                return encoders;
            }

            void read_home() {
                yInfo() << "read home";
                command_home = read();
            }

            void set(Vector command) {
                pos->positionMove(command.data());
            }

            void home() {
                yInfo() << "set home";
                set( command_home );
            }
    };

    BodyPart arm_l, arm_r, torso, head;

    /**************************************************************************/
    bool configure(ResourceFinder& rf) override {
        const string name = "micro_robot_reset";

        arm_l.open( "/icubSim/right_arm", name );
        arm_r.open( "/icubSim/left_arm", name );
        head.open( "/icubSim/head", name );
        torso.open( "/icubSim/torso", name );

        // Property hand_r_options;
        // hand_r_options.put("device", "remote_controlboard");
        // hand_r_options.put("local", "/"+name+"/hand_r");
        // hand_r_options.put("remote", "/icubSim/right_arm");
        // hand_r.open(hand_r_options);

        // IPositionControl *hand_r_pos;
        // IEncoders *hand_r_encs;
        // robotDevice.view(hand_r_pos);
        // robotDevice.view(hand_r_encs);

        // Property hand_l_options;
        // hand_l_options.put("device", "remote_controlboard");
        // hand_l_options.put("local", "/"+name+"/hand_l");
        // hand_l_options.put("remote", "/icubSim/left_arm");

        // Property gaze_options;
        // gaze_options.put("device", "gazecontrollerclient");
        // gaze_options.put("local", "/"+name+"/gaze");
        // gaze_options.put("remote", "/iKinGazeCtrl");

        // Property torso_options;
        // torso_options.put("device", "remote_controlboard");
        // torso_options.put("local", "/"+name+"/gaze");
        // torso_options.put("remote", "/icubSim/torso");

        // IPositionControl *pos;
        // IEncoders *encs;
        // robotDevice.view(pos);
        // robotDevice.view(encs);


        // // enable position control of the fingers
        // {
        //     vector<PolyDriver*> polys({&hand_r, &hand_l});
        //     for (auto poly:polys) {
        //         IControlMode* imod;
        //         poly->view(imod);
        //         imod->setControlModes(fingers.size(), fingers.data(), vector<int>(fingers.size(), VOCAB_CM_POSITION).data());
        //     }
        // }

        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);

        return true;
    }

    /**************************************************************************/
    bool home() override {
        arm_l.home();
        arm_r.home();
        torso.home();
        head.home();
        return true;
    }

    /**************************************************************************/
    // bool home() override {
    //     // home gazing
    //     IGazeControl* igaze;
    //     gaze.view(igaze);
    //     igaze->lookAtAbsAnglesSync({0., -50., 10.});



    //     // home arms
    //     {
    //         Vector x{-.25, .3, .1};
    //         vector<PolyDriver*> polys({&arm_r, &arm_l});
    //         ICartesianControl* iarm;
    //         for (auto poly:polys) {                
    //             poly->view(iarm);
    //             iarm->goToPositionSync(x);                
    //             x[1] = -x[1];
    //         }
    //         // wait only for the last arm
    //         // iarm->waitMotionDone();
    //     }

    //     igaze->waitMotionDone();
    //     return true;
    // }

    // bool reach() {
    //         vector<PolyDriver*> polys({&arm_r});
    //         ICartesianControl* iarm;
    //         for (auto poly:polys) {                
    //             poly->view(iarm);
    //             iarm->goToPositionSync(reach_x);                
    //         }
    //         // wait only for the last arm
    //         //iarm->waitMotionDone();
    //         iarm->waitMotionDone(.1, 3.);
    //         return true;
    // }

//     bool grasp() override {

//         const Vector sqCenter{sqParams.get(0).asFloat64(),
//                               sqParams.get(1).asFloat64(),
//                               sqParams.get(2).asFloat64()};

//         // set up the hand pre-grasp configuration
//         IControlLimits* ilim;
//         hand_r.view(ilim);
//         double pinkie_min, pinkie_max;
//         ilim->getLimits(15, &pinkie_min, &pinkie_max);
//         const vector<double> pregrasp_fingers_posture{60., 80., 0., 0., 0., 0., 0., 0., pinkie_max};

//         // keep gazing at the object
//         IGazeControl* igaze;
//         gaze.view(igaze);
//         igaze->setTrackingMode(true);
//         igaze->lookAtFixationPoint(sqCenter);

//         // apply cardinal points grasp algorithm
//         ICartesianControl* iarm;
//         arm_r.view(iarm);
//         auto grasper_r = make_shared<CardinalPointsGrasp>(CardinalPointsGrasp("right", pregrasp_fingers_posture));
//         const auto candidates_r = grasper_r->getCandidates(sqParams, iarm);

//         arm_l.view(iarm);
//         auto grasper_l = make_shared<CardinalPointsGrasp>(CardinalPointsGrasp("left", pregrasp_fingers_posture));
//         const auto candidates_l = grasper_l->getCandidates(sqParams, iarm);

//         // aggregate right and left arms candidates
//         auto candidates = candidates_r.first;
//         candidates.insert(candidates.end(), candidates_l.first.begin(), candidates_l.first.end());
//         std::sort(candidates.begin(), candidates.end(), CardinalPointsGrasp::compareCandidates);

//         // some safety checks
//         if (candidates.empty()) {
//             yError() << "No good grasp candidates found!";
//             //lookAtDeveloper();
//             //shrug();
//             return false;
//         }

//         // extract relevant info
//         const auto& best = candidates[0];
//         const auto& type = get<0>(best);
//         const auto& T = get<2>(best);

// //        viewer->showCandidates(candidates);

//         // select arm corresponing to the best candidate
//         shared_ptr<CardinalPointsGrasp> grasper;
//         IPositionControl* ihand;
//         int context;
//         if (type == "right") {
//              grasper = grasper_r;
//              hand_r.view(ihand);
//              arm_r.view(iarm);
//              context = candidates_r.second;
//         } else {
//              grasper = grasper_l;
//              hand_l.view(ihand);
//              arm_l.view(iarm);
//              context = candidates_l.second;
//         }

//         // target pose that allows grasping the object
//         const auto x = T.getCol(3).subVector(0, 2);
//         const auto o = dcm2axis(T);

//         // enable the context used by the algorithm
//         iarm->stopControl();
//         iarm->restoreContext(context);
//         iarm->setInTargetTol(.001);
//         iarm->setTrajTime(1.);

//         yInfo() << "put the hand in the pre-grasp configuration";

//         // put the hand in the pre-grasp configuration
//         // ihand->setRefAccelerations(fingers.size(), fingers.data(), vector<double>(fingers.size(), numeric_limits<double>::infinity()).data());
//         // ihand->setRefSpeeds(fingers.size(), fingers.data(), vector<double>{60., 60., 60., 60., 60., 60., 60., 60., 200.}.data());
//         // ihand->positionMove(fingers.size(), fingers.data(), pregrasp_fingers_posture.data());
//         // auto done = false;
//         // while (!done) {
//         //     Time::delay(1.);
//         //     ihand->checkMotionDone(fingers.size(), fingers.data(), &done);
//         // };

//         //yInfo() << "reach for the pre-grasp pose....";

//         // reach for the pre-grasp pose
//         //const auto dir = x - sqCenter;
//         //iarm->goToPoseSync(x + .05 * dir / norm(dir), o);
//         //iarm->waitMotionDone(.1, 3.);
        
//         yInfo() << "reach for the object";

//         // reach for the object
//         iarm->goToPoseSync(x, o);
//         //iarm->waitMotionDone(.1, 3.);


//         ihand->setRefAccelerations(fingers.size(), fingers.data(), vector<double>(fingers.size(), numeric_limits<double>::infinity()).data());
//         ihand->setRefSpeeds(fingers.size(), fingers.data(), vector<double>{60., 60., 60., 60., 60., 60., 60., 60., 200.}.data());
//         ihand->positionMove(fingers.size(), fingers.data(), pregrasp_fingers_posture.data());
//         auto done = false;
//         while (!done) {
//             Time::delay(1.);
//             ihand->checkMotionDone(fingers.size(), fingers.data(), &done);
//         };



//         yInfo() << "close fingers";

//         // close fingers
//         ihand->positionMove(fingers.size(), fingers.data(), vector<double>{60., 80., 40., 35., 40., 35., 40., 35., pinkie_max}.data());


//         return true;

//     }

    /**************************************************************************/
//     bool grasp_full() {

        
//         const Vector sqCenter{sqParams.get(0).asFloat64(),
//                               sqParams.get(1).asFloat64(),
//                               sqParams.get(2).asFloat64()};


//         // keep gazing at the object
//         IGazeControl* igaze;
//         gaze.view(igaze);
//         igaze->setTrackingMode(true);
//         igaze->lookAtFixationPoint(sqCenter);

//         // set up the hand pre-grasp configuration
//         IControlLimits* ilim;
//         hand_r.view(ilim);
//         double pinkie_min, pinkie_max;
//         ilim->getLimits(15, &pinkie_min, &pinkie_max);
//         const vector<double> pregrasp_fingers_posture{60., 80., 0., 0., 0., 0., 0., 0., pinkie_max};

//         // apply cardinal points grasp algorithm
//         ICartesianControl* iarm;
//         arm_r.view(iarm);
//         auto grasper_r = make_shared<CardinalPointsGrasp>(CardinalPointsGrasp("right", pregrasp_fingers_posture));
//         const auto candidates_r = grasper_r->getCandidates(sqParams, iarm);

//         arm_l.view(iarm);
//         auto grasper_l = make_shared<CardinalPointsGrasp>(CardinalPointsGrasp("left", pregrasp_fingers_posture));
//         const auto candidates_l = grasper_l->getCandidates(sqParams, iarm);

//         // aggregate right and left arms candidates
//         auto candidates = candidates_r.first;
//         candidates.insert(candidates.end(), candidates_l.first.begin(), candidates_l.first.end());
//         std::sort(candidates.begin(), candidates.end(), CardinalPointsGrasp::compareCandidates);

//         // some safety checks
//         if (candidates.empty()) {
//             yError() << "No good grasp candidates found!";
//             //lookAtDeveloper();
//             //shrug();
//             return false;
//         }

//         // extract relevant info
//         const auto& best = candidates[0];
//         const auto& type = get<0>(best);
//         const auto& T = get<2>(best);

// //        viewer->showCandidates(candidates);

//         // select arm corresponing to the best candidate
//         shared_ptr<CardinalPointsGrasp> grasper;
//         IPositionControl* ihand;
//         int context;
//         if (type == "right") {
//              grasper = grasper_r;
//              hand_r.view(ihand);
//              arm_r.view(iarm);
//              context = candidates_r.second;
//         } else {
//              grasper = grasper_l;
//              hand_l.view(ihand);
//              arm_l.view(iarm);
//              context = candidates_l.second;
//         }

//         // target pose that allows grasping the object
//         const auto x = T.getCol(3).subVector(0, 2);
//         const auto o = dcm2axis(T);

//         // enable the context used by the algorithm
//         iarm->stopControl();
//         iarm->restoreContext(context);
//         iarm->setInTargetTol(.001);
//         iarm->setTrajTime(1.);

//         yInfo() << "put the hand in the pre-grasp configuration";

//         // put the hand in the pre-grasp configuration
//         ihand->setRefAccelerations(fingers.size(), fingers.data(), vector<double>(fingers.size(), numeric_limits<double>::infinity()).data());
//         ihand->setRefSpeeds(fingers.size(), fingers.data(), vector<double>{60., 60., 60., 60., 60., 60., 60., 60., 200.}.data());
//         ihand->positionMove(fingers.size(), fingers.data(), pregrasp_fingers_posture.data());
//         auto done = false;
//         while (!done) {
//             Time::delay(1.);
//             ihand->checkMotionDone(fingers.size(), fingers.data(), &done);
//         };

//         yInfo() << "reach for the pre-grasp pose....";

//         // reach for the pre-grasp pose
//         const auto dir = x - sqCenter;
//         iarm->goToPoseSync(x + .05 * dir / norm(dir), o);
//         iarm->waitMotionDone(.1, 3.);
        
//         yInfo() << "reach for the object";

//         // reach for the object
//         iarm->goToPoseSync(x, o);
//         iarm->waitMotionDone(.1, 3.);

//         yInfo() << "close fingers";

//         // close fingers
//         ihand->positionMove(fingers.size(), fingers.data(), vector<double>{60., 80., 40., 35., 40., 35., 40., 35., pinkie_max}.data());

//         yInfo() << "delay for fingers";

//         // give enough time to adjust the contacts
//         Time::delay(1.); // was 5.

//         yInfo() << "lift...";

//         // lift up the object
//         const auto lift = x + Vector{0., 0., .1};
//         igaze->lookAtFixationPoint(lift);
//         iarm->goToPoseSync(lift, o);
//         iarm->waitMotionDone(.1, 3.);

//         //lookAtDeveloper();
//         return true;
//     }

    /**************************************************************************/
    double getPeriod() override {
        return 0.010;
    }

    /**************************************************************************/
    bool updateModule() override {
        // if( auto* sqParams0 = requestedSuperquadric.read(false) ) {
        //     yInfo() << "custom sqParams request";
        //     if( sqParams0->size() == 7 ) {
        //         sqParams.copy( *sqParams0 );
        //         yInfo() << "done, grasping...";
        //         grasp();
        //         yInfo() << "finished grasp!";
        //     } else if( sqParams0->size() == 3 ) {
        //         yInfo() << "3";
        //         reach_x[0] = sqParams0->get(0).asFloat64();
        //         reach_x[1] = sqParams0->get(1).asFloat64();
        //         reach_x[2] = sqParams0->get(2).asFloat64();
        //         yInfo() << "reaching...";
        //         reach();
        //         yInfo() << "finished reach!";
        //     } else {
        //         yInfo() << "... but wrong size";
        //     }


        // }

        return true;
    }

    bool interruptModule() override {

        // interrupt blocking read
        // sqPort.interrupt();
        return true;
    }

    /**************************************************************************/
    bool close() override {
        // restore default contexts
        // IGazeControl* igaze;
        // gaze.view(igaze);
        // igaze->stopControl();
        // igaze->restoreContext(0);

        // vector<PolyDriver*> polys({&arm_r, &arm_l});
        // for (auto poly:polys) {
        //     ICartesianControl* iarm;
        //     poly->view(iarm);
        //     iarm->stopControl();
        //     iarm->restoreContext(0);
        // }

        // rpcPort.close();
        // sqPort.close();

        // gaze.close();
        // arm_r.close();
        // arm_l.close();
        // hand_r.close();
        // hand_l.close();
        // return true;
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

    GrasperModule module;
    return module.runModule(rf);
}
