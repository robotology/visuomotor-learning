# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Author: NGUYEN Dong Hai Phuong <phuong.nguyen@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(reachObjMultiDemo)

set(PROJECTNAME collectData)
project(${PROJECTNAME})
aux_source_directory(. SRC_LIST)

set (include_files	${CMAKE_CURRENT_SOURCE_DIR}/collectData.h)

set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/collectData.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/collectDataModule.cpp)

set(idl_files    collectData.thrift)

yarp_add_idl(IDL_GEN_FILES collectData.thrift)

source_group("IDL Files" FILES ${idl_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECTNAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files} ${IDL_GEN_FILES} ${idl_files})# ${local_lib_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib iKin)

install (TARGETS ${PROJECTNAME} DESTINATION bin)

