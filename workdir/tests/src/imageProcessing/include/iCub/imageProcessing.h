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
 * @file imageProcessing.h
 * @brief Simple module that repeats an RGB image received via input connection (eg.tcp) to an output connection (eg mcast).
 * The module can be used to transfer network congestion.
 */

#ifndef _R_MODULE_H_
#define _R_MODULE_H_

/** 
 *
 * \defgroup icub_repeater repeater
 * @ingroup icub_logpolarAttention
 *
 * This is a module that receives the RGB image from input connection and sends it back to output connection. The purpose
 * of the module is to shift the point of congestion in a network.
 * 
 *
 
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c repeater.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c repeater/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c repeater \n 
 *   specifies the name of the repeater (used to form the stem of repeater port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /repeater \n
 *    This port is used to change the parameters of the repeater at run time or stop the repeater. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other repeaters but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /repeater
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /repeater/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /repeater \n
 *    see above
 *
 *  - \c /repeater/image:o \n
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the repeater 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myInputPort; \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myOutputPort;       
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c repeater.ini  in \c $ICUB_ROOT/app/repeater/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/repeater/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>repeater --name repeater --context repeater/conf --from repeater.ini --robot icub</tt>
 *
 * \author Shashank
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * 
 */


#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
//within project includes  
#include <iCub/imageProcessingThread.h>



class imageProcessing:public yarp::os::RFModule {

    
    std::string moduleName;                  // name of the module
    std::string robotName;                   // name of the robot 
    std::string robotPortName;               // name of robot port
    std::string inputPortName;               // name of the input port for events
    std::string outputPortName;              // name of output port
    std::string handlerPortName;             // name of handler port
    std::string configFile;                  // name of the configFile that the resource Finder will seek
    yarp::os::Semaphore mutex;
    yarp::os::Port handlerPort;              // a port to handle messages 
    /*  */
    imageProcessingThread *rThread;             // pointer to a new thread to be created and started in configure() and stopped in close()
    double verticalOffset;

public:
    /**
    *  configure all the repeater parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    bool configure(yarp::os::ResourceFinder &rf); 
   
    /**
    *  interrupt, e.g., the ports 
    */
    bool interruptModule();                    

    /**
    *  close and shut down the repeater
    */
    bool close();

    /**
    *  to respond through rpc port
    * @param command reference to bottle given to rpc port of module, alongwith parameters
    * @param reply reference to bottle returned by the rpc port in response to command
    * @return bool flag for the success of response else termination of module
    */
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /**
    *  implemented to define the periodicity of the module
    */
    double getPeriod();

    /**
    *  unimplemented
    */ 
    bool updateModule();
};


#endif // __R_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

