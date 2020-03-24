# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Author: NGUYEN Dong Hai Phuong (inspried from skinEventsAggregator of Matej Hoffmann PPS)
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.5)
project(skinEventsSampler)

file(GLOB source *.cpp)
file(GLOB header *.h)

#include_directories(${periPersonalSpace_INCLUDE_DIRS})

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${ICUB_INCLUDE_DIRS})

add_executable(skinEventsSampler ${source} ${header})

target_link_libraries(skinEventsSampler skinDynLib ${YARP_LIBRARIES})

install(TARGETS skinEventsSampler DESTINATION bin)

