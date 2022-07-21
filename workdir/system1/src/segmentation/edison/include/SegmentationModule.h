/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_SERVICE_SEGMENTATIONMODULE_H
#define YARP_THRIFT_GENERATOR_SERVICE_SEGMENTATIONMODULE_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <SpeedUpLevelComm.h>

class SegmentationModule :
        public yarp::os::Wire
{
public:
    // Constructor
    SegmentationModule();

    virtual void set_sigmaS(const double newValue);

    virtual void set_sigmaR(const double newValue);

    virtual void set_minRegion(const double newValue);

    virtual void set_gradWindRad(const double newValue);

    virtual void set_threshold(const double newValue);

    virtual void set_mixture(const double newValue);

    virtual void set_speedup(const SpeedUpLevelComm newSpeedLevel);

    virtual double get_sigmaS();

    virtual double get_sigmaR();

    virtual double get_minRegion();

    virtual double get_gradWindRad();

    virtual double get_threshold();

    virtual double get_mixture();

    virtual SpeedUpLevelComm get_speedup();

    // help method
    virtual std::vector<std::string> help(const std::string& functionName = "--all");

    // read from ConnectionReader
    bool read(yarp::os::ConnectionReader& connection) override;
};

#endif // YARP_THRIFT_GENERATOR_SERVICE_SEGMENTATIONMODULE_H
