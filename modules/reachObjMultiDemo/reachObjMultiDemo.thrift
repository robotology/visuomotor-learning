# Copyright: (C) 2018 iCub Facility - Istituto Italiano di Tecnologia
# Author: NGUYEN Dong Hai Phuong <phuong.nguyen@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#
# reachObjMultiDemo.thrift

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
* reachObjMultiDemo_IDL
*
* IDL Interface to \ref motorBabbling services.
*/
service reachObjMultiDemo_IDL
{
  /**
  * Move arms to home positions
  */
  bool home_arms();

  /**
  * Move all to home positions
  */
  bool home_all();

  /**
  * Reach the ball
  * @param _useCalib
  * @return true/false on success/failure.
  */
  bool reach(1:bool _useCalib);

  /**
  * Babble then reach the object
  * @param _id
  * @param _part
  * @return true/false on success/failure.
  */
  bool babble_then_reach(1:i32 _id, 2:string _part);

  /**
  * Resume the current babbling action after stop()
  * @return true/false on success/failure.
  **/
  bool resume();

  /**
  * Sets the tilt hand angle for reaching.
  * @param _ang value of babbling.
  * @return true/false on success/failure.
  */
  bool set_tilt_angle(1:double _ang);

  /**
  * Gets the tilt hand angle for reaching.
  * @return the current tiltAngle value.
  */
  double get_tilt_angle();

  /**
  * Sets the yaw hand angle for reaching.
  * @param _ang value of babbling.
  * @return true/false on success/failure.
  */
  bool set_yaw_angle(1:double _ang);

  /**
  * Gets the yaw hand angle for reaching.
  * @return the current tiltAngle value.
  */
  double get_yaw_angle();

  /**
  * Enable to use hand angle for reaching.
  * @return true/false on success/failure.
  */
  bool enable_hand_angle();

  /**
  * Disable to use hand angle for reaching.
  * @return true/false on success/failure.
  */
  bool disable_hand_angle();
   
  /**
  * Stop the current babbling action
  * @return true/false on success/failure.
  **/
  bool stop();


}
