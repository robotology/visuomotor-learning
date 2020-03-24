# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Author: NGUYEN Dong Hai Phuong <phuong.nguyen@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#
# collectData.thrift

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
* collectData_IDL
*
* IDL Interface to \ref motorBabbling services.
*/
service collectData_IDL
{
  /**
  * Babble then reach the object
  * @param _id taxel id in the part, -1 for all
  * @param _part name of part, which can be both(b)/hand(h)/arm(a)
  * @param _nb_object number of generated objects
  * @return true/false on success/failure.
  */
  bool babble_then_reach(1:i32 _id, 2:string _part, 3:i32 _nb_object);

  
}
