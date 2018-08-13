/*
 * Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Nguyen Dong Hai Phuong <phuong.nguyen@iit.it>
 * website: www.robotcub.org
 * author website: https://github.com/towardthesea
 *
 * This module is inspired from icub-hri/babbling of Martina Zambelli.
 * Please find the github.com/robotology/icub-hri website for the details.
 *
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
 * Public License for more details.
*/

#ifndef MOTORBABBLING_H
#define MOTORBABBLING_H

#include <cstdlib>
#include <ctime>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include "motorBabbling_IDL.h"

class motorBabbling : public yarp::os::RFModule, public motorBabbling_IDL
{
protected:
    yarp::os::RpcServer             rpcPort;

    yarp::dev::IPositionControl*    posArm;
    yarp::dev::IVelocityControl*    velArm;
    yarp::dev::ITorqueControl *     itrqArm;
    yarp::dev::IEncoders*           encsArm;
    yarp::dev::IControlMode*        iCtrlArm;
    yarp::dev::IControlLimits*      iCtrlLimArm;

    yarp::dev::IPositionControl*    posLeftArm;
    yarp::dev::IVelocityControl*    velLeftArm;
    yarp::dev::ITorqueControl *     itrqLeftArm;
    yarp::dev::IEncoders*           encsLeftArm;
    yarp::dev::IControlMode*        iCtrlLeftArm;
    yarp::dev::IControlLimits*      iCtrlLimLeftArm;

    yarp::dev::IPositionControl*    posRightArm;
    yarp::dev::IVelocityControl*    velRightArm;
    yarp::dev::ITorqueControl*      itrqRightArm;
    yarp::dev::IEncoders*           encsRightArm;
    yarp::dev::IControlMode*        iCtrlRightArm;
    yarp::dev::IControlLimits*      iCtrlLimRightArm;

    yarp::dev::IGazeControl*        igaze;

    yarp::dev::ICartesianControl   *iCartCtrlL;
    yarp::dev::ICartesianControl   *iCartCtrlR;
    yarp::dev::ICartesianControl   *iCartCtrl;

    yarp::dev::PolyDriver           leftArmDev;
    yarp::dev::PolyDriver           rightArmDev;
    yarp::dev::PolyDriver           headDev;

    yarp::dev::PolyDriver           leftCartDev;
    yarp::dev::PolyDriver           rightCartDev;

    yarp::sig::Vector               encodersLeftArm, encodersRightArm;
    yarp::os::BufferedPort<yarp::os::Bottle>    eePoseOutPort;             //!< buffered port of dumped End-effector pose output
    yarp::os::Stamp                 ts;


    std::string part;               //!< Should be "left_arm" or "right_arm"
    std::string robot;              //!< Should be "icub" or "icubSim"
    std::string part_babbling;      //!< Whether to do babbling with the whole "arm" or "hand" (should be either one or the other)
    int         single_joint;       //!< Which joint to do babbling with in case it is a single joint
    double      period;
    std::string name;

    double      freq;               //!< Frequency for sin wave
    double      amp;                //!< Amplitude for sin wave
    double      ampH;               //!< Amplitude for head
    double      duration;           //!< Duration for the babbling
    double      startBabblingTime;  //!< Time at which the babbling starts
    bool        isBabbling;         //!< Flag for babbling to run
    bool        interruptBabbling;  //!< Flag to interrupt the babbling
    bool        moveHeadOnly;       //!< Flag to babbling only the head
    bool        keepStatic;         //!< Flag to keep arm static during babbling, only move to initial position

    yarp::sig::Vector               fixatePos;  //!< fixate position of iCub eye, which is use to calculate the start position of hand

    int32_t     nbRepeat;           //!< Number of babbling time for 1 arm, set by auto_babbling_arm
    int32_t     curRepeat;          //!< Counter for auto_babbling_arm
    std::string autoArmName;        //!< Name of arm in auto babbling mode

    yarp::sig::Vector   new_command_arm;
    yarp::sig::Vector   new_command_head;
    double              start_command_arm[16];  //!< Start command for the 16 arm joints
    double              away_command_arm[16];   //!< Away command for the 16 arm joints
    yarp::sig::Vector   start_command_head;     //!< Target for head in start position
    double              neck_range_min[3];
    double              neck_range_max[3];
    double              minLimArm[16];
    double              maxLimArm[16];

    bool    initArm(const std::string &part, yarp::dev::PolyDriver &armDev);
    bool    init_left_arm(); //!< Create PolyDriver for left arm
    bool    init_right_arm(); //!< Create PolyDriver for left arm
    bool    initRobot();
    bool    moveHeadToStartPos(bool ranPos);
    bool    moveHeadRandomly();
    bool    moveHeadToCentralPos();
    bool    moveArmToStartPos(const std::string &partName, bool home);
    bool    moveArmAway(const std::string &partName);
    bool    gotoStartPos(bool moveAway);
    void    babblingCommands(const double &t, int j_idx);
    bool    startBabbling();

public:
    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    attach(yarp::os::RpcServer &source);
    double  getPeriod();
    bool    updateModule();

    /*******************************************************************/
    //Thrift
    bool auto_babble_arm(const std::string &armName, const int32_t _nbRepeat)
    {
        if(_nbRepeat > 0)
        {
//            moveHeadToStartPos(false);
            yarp::os::Time::delay(0.5);
            nbRepeat = _nbRepeat;
            curRepeat = 0;
            autoArmName = armName;
            return true;
        }
        else
            return false;
    }

    bool babble_arm(const std::string &armName)
    {
        if (armName == "left" || armName == "right")
        {
            single_joint = -1;
            part_babbling = "arm";
            part = armName;
            startBabbling();
            return true;
        }
        else
            return false;
    }

    bool auto_babble_head(const std::string &armName, const int32_t _nbRepeat)
    {
        if(_nbRepeat > 0)
        {
            yarp::os::Time::delay(0.5);
            nbRepeat = _nbRepeat;
            curRepeat = 0;
            autoArmName = armName;
            moveHeadOnly = true;
            return true;
        }
        else
            return false;
    }

    bool babble_head(const std::string &side)
    {
        if (side == "left" || side == "right")
        {
            part = side;
//            bool ok = startBabblingHead();
            bool ok = gotoStartPos(true);
            if (ok)
                yDebug("[%s] Moved all arms away sucessfully!!",name.c_str());
            else
                yDebug("[%s] Failed moving all arms away!!",name.c_str());
            isBabbling = true;
            moveHeadOnly = true;
            return ok;
        }
        else
            return false;
    }

    bool babble_hand(const std::string &handName)
    {
        if (handName == "left" || handName == "right")
        {
            single_joint = -1;
            part_babbling = "hand";
            part = handName;
            startBabbling();
            return true;
        }
        else
            return false;
    }

    bool babble_joint(const std::string &armName, const int32_t jointNumber)
    {
        if (armName == "left" || armName == "right")
        {
            if (jointNumber < 16 && jointNumber >= 0)
            {
                single_joint = jointNumber;
                part_babbling = "arm";
                part = armName;
                startBabbling();
                return true;
            }
            else
                return false;
        }
        else
            return false;
    }

    bool home_arms()
    {
        bool ok = false;
        ok = moveArmToStartPos("left", true);
        ok = ok & moveArmToStartPos("right", true);
        if (ok)
            yDebug("[%s] Moved all arms home sucessfully!!",name.c_str());
        else
            yDebug("[%s] Failed moving all arms home!!",name.c_str());
        return ok;
    }

    bool home_all()
    {
        bool ok = false;
        ok = moveHeadToCentralPos();
        ok = ok & home_arms();
        if (ok)
            yDebug("[%s] Moved all parts home sucessfully!!",name.c_str());
        else
            yDebug("[%s] Failed moving all parts home!!",name.c_str());
        return ok;
    }

    bool set_amp_head(const double _amp)
    {
        if (_amp>=0.0)
        {
            ampH = _amp;
            return true;
        }
        else
            return false;
    }

    double get_amp_head()
    {
        return ampH;
    }

    bool set_amp(const double _amp)
    {
        if (_amp>=0.0)
        {
            amp = _amp;
            return true;
        }
        else
            return false;
    }

    double get_amp()
    {
        return amp;
    }

    bool set_freq(const double _freq)
    {
        if (_freq>=0.0)
        {
            freq = _freq;
            return true;
        }
        else
            return false;
    }

    double get_freq()
    {
        return freq;
    }

    bool resume()
    {
        return interruptBabbling = false;
    }

    bool stop()
    {
        return interruptBabbling = true;
    }

};

#endif // MOTORBABBLING_H
