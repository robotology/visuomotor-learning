# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: NGUYEN Dong Hai Phuong <phuong.nguyen@iit.it>
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#
# motorBabbling.thrift

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
* motorBabbling_IDL
*
* IDL Interface to \ref motorBabbling services.
*/
service motorBabbling_IDL
{
  /**
  * Start babbling with a joint in an arm
  * @param jointNumber: integer value for joint
  * @param armName: string value for arm
  * @return true/false on success/failure.
  */
  bool babble_joint(1:string armName, 2:i32 jointNumber);

  /**
  * Start babbling with an hand
  * @param handName: string value for hand
  * @return true/false on success/failure.
  */
  bool babble_hand(1:string handName);

  /**
  * Start babbling only the head
  * @param side: string value for left/right side to mainly move the head
  * @return true/false on success/failure.
  */
  bool babble_head(1:string side);

  /**
  * Start babbling with head automatically in number of times
  * @param side: string value for left/right side to mainly move the head
  * @param _nbRepeat: integer value for number of times
  * @return true/false on success/failure.
  */
  bool auto_babble_head(1:string side, 2:i32 _nbRepeat);

  /**
  * Start babbling with an arm
  * @param armName: string value for arm
  * @return true/false on success/failure.
  */
  bool babble_arm(1:string armName);

  /**
  * Start babbling with an arm automatically in number of times
  * @param armName: string value for arm
  * @param _nbRepeat: integer value for number of times
  * @return true/false on success/failure.
  */
  bool auto_babble_arm(1:string armName, 2:i32 _nbRepeat);

  /**
  * Move arms to home positions
  */
  bool home_arms();

  /**
  * Move all to home positions
  */
  bool home_all();

  /**
  * Sets head babbling amplitude.
  * @param _amp value of babbling.
  * @return true/false on success/failure.
  */
  bool set_amp_head(1:double _amp);

  /**
  * Gets head the babbling amplitude.
  * @return the current ampH value.
  */
  double get_amp_head();

  /**
  * Sets babbling amplitude.
  * @param _amp value of babbling.
  * @return true/false on success/failure.
  */
  bool set_amp(1:double _amp);

  /**
  * Gets the babbling amplitude.
  * @return the current amp value.
  */
  double get_amp();

  /**
  * Sets babbling frequency.
  * @param _freq value of babbling.
  * @return true/false on success/failure.
  */
  bool set_freq(1:double _freq);

  /**
  * Gets the babbling frequency.
  * @return the current freq value.
  */
  double get_freq();

  /**
  * Resume the current babbling action after stop()
  * @return true/false on success/failure.
  **/
  bool resume();
   
  /**
  * Stop the current babbling action
  * @return true/false on success/failure.
  **/
  bool stop();


}
