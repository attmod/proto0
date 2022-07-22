/*
 * SPDX-FileCopyrightText: 2006-2021 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Autogenerated by Thrift Compiler (0.14.1-yarped)
//
// This is an automatically generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_ENUM_SPEEDUPLEVELCOMM_H
#define YARP_THRIFT_GENERATOR_ENUM_SPEEDUPLEVELCOMM_H

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

enum SpeedUpLevelComm : int32_t
{
    NO_SPEEDUP_COMM = 0,
    MED_SPEEDUP_COMM = 1,
    HIGH_SPEEDUP_COMM = 2
};

class SpeedUpLevelCommConverter
{
public:
    static int32_t fromString(const std::string& input);
    static std::string toString(int32_t input);
};

#endif // YARP_THRIFT_GENERATOR_ENUM_SPEEDUPLEVELCOMM_H