# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: NGUYEN Dong Hai Phuong <phuong.nguyen@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(motorBabbling)

set(PROJECTNAME motorBabbling)
project(${PROJECTNAME})
aux_source_directory(. SRC_LIST)

include(YarpIDL)

set (include_files	${CMAKE_CURRENT_SOURCE_DIR}/motorBabbling.h)

set (source_files 	${CMAKE_CURRENT_SOURCE_DIR}/motorBabbling.cpp
                        ${CMAKE_CURRENT_SOURCE_DIR}/babblingModule.cpp)

set(idl_files    motorBabbling.thrift)

yarp_add_idl(IDL_GEN_FILES motorBabbling.thrift)

source_group("IDL Files" FILES ${idl_files})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${ICUB_INCLUDE_DIRS})

add_definitions(-D_USE_MATH_DEFINES)

add_executable(${PROJECTNAME} ${SRC_LIST} ${lib_files} ${source_files} ${include_files} ${IDL_GEN_FILES} ${idl_files})# ${local_lib_files})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)

install (TARGETS ${PROJECTNAME} DESTINATION bin)

