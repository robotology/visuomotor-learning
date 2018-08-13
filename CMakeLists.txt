# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Nguyen Dong Hai Phuong <phuong.nguyen@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

cmake_minimum_required(VERSION 2.8)

set(PROJECTNAME visuomotor-learning)
project(${PROJECTNAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

aux_source_directory(. SRC_LIST)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
#find_package(icubclient)
#find_package(OpenCV REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

include(YarpInstallationHelpers)   # pick up yarp's cmake scripts

set(main_project_path ${CMAKE_CURRENT_SOURCE_DIR})


add_subdirectory(app)
add_subdirectory(modules)

icubcontrib_add_uninstall_target()

