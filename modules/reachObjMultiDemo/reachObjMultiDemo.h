#ifndef REACHOBJMULTIDEMO_H
#define REACHOBJMULTIDEMO_H

#include <cstdlib>
#include <ctime>
#include <vector>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>

#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/math.h>

#include <gsl/gsl_math.h>

#include "reachObjMultiDemo_IDL.h"

class reachObjMultiDemo : public yarp::os::RFModule, public reachObjMultiDemo_IDL
{
protected:
    yarp::os::RpcServer             rpcPort;

    yarp::dev::IPositionControl*    posArm;
    yarp::dev::IVelocityControl*    velArm;
    yarp::dev::ITorqueControl *     itrqArm;
    yarp::dev::IEncoders*           encsArm;
    yarp::dev::IControlMode*        iCtrlArm;
    yarp::dev::IControlLimits*      iCtrlLimArm;

    yarp::dev::IPositionControl*    posRightArm;
    yarp::dev::IVelocityControl*    velRightArm;
    yarp::dev::ITorqueControl*      itrqRightArm;
    yarp::dev::IEncoders*           encsRightArm;
    yarp::dev::IControlMode*        iCtrlRightArm;
    yarp::dev::IControlLimits*      iCtrlLimRightArm;

    yarp::dev::IPositionControl*    posTorso;
    yarp::dev::IVelocityControl*    velTorso;
    yarp::dev::ITorqueControl*      itrqTorso;
    yarp::dev::IEncoders*           encsTorso;
    yarp::dev::IControlMode*        iCtrlTorso;
    yarp::dev::IControlLimits*      iCtrlLimTorso;

    iCub::iKin::iCubArm*            arm;
    yarp::sig::Vector               qA;

    yarp::dev::IGazeControl*        igaze;
    int                             contextGaze;
    yarp::dev::ICartesianControl   *iCartCtrlR;
    yarp::dev::ICartesianControl   *iCartCtrl;

    yarp::dev::PolyDriver           torsoDev;
    yarp::dev::PolyDriver           rightArmDev;
    yarp::dev::PolyDriver           headDev;

    yarp::dev::PolyDriver           rightCartDev;

    yarp::sig::Vector               encodersRightArm;
    yarp::sig::Vector               encodersTorso;

    std::string part;               //!< Should be "left_arm" or "right_arm"
    std::string robot;              //!< Should be "icub" or "icubSim"
    double      period;
    std::string name;
    bool        changeTipFrame;     //!<
    bool        useFakeTarget;
    bool        useHandAngle;
    yarp::sig::Vector               fakeTarget;
    double                          tiltAngle, yawAngle;


    yarp::sig::Vector   new_command_arm;
    yarp::sig::Vector   new_command_head;
    double              start_command_arm[16];  //!< Start command for the 16 arm joints
    double              away_command_arm[16];   //!< Away command for the 16 arm joints
    yarp::sig::Vector   start_command_head;     //!< Target for head in start position
    double              neck_range_min[3];
    double              neck_range_max[3];
    double              minLimArm[16];
    double              maxLimArm[16];

    bool                hasCommand;
    bool                hasTarget;

    yarp::os::BufferedPort<yarp::os::Bottle>    cmdJointsPort;
    yarp::os::Bottle                            *cmdJointsBottle;
    yarp::sig::Vector                           cmdJointsAng;

    yarp::os::BufferedPort<yarp::os::Bottle>    objPosDimPort;
    yarp::os::Bottle                            *objPoseDimBottle;
    yarp::sig::Vector                           objPoseDim;

    yarp::os::BufferedPort<yarp::os::Bottle>    tactilePort;
    yarp::os::Bottle                            *tactileBottle;


    yarp::os::BufferedPort<yarp::os::Bottle>    targetDumpedData;             //!< buffered port of dumped target data
    yarp::os::Stamp                             ts;

    yarp::sig::Vector                           joints_arm_min, joints_arm_max;

    yarp::os::RpcClient     rpcToMotorBabbling;
    yarp::os::RpcClient     rpcToLearning;

    bool    init_right_arm(); //!< Create PolyDriver for left arm
    bool    initRobot();
//    bool    moveHeadToStartPos(bool ranPos);
    bool    moveHeadToCentralPos();
    bool    moveArmToStartPos(const std::string &partName);
    bool    moveArmAway(const std::string &partName);
    bool    gotoStartPos(bool moveAway);
    bool    reachTarget();

    void    updateArmChain(const yarp::sig::Vector &q, yarp::sig::Vector &xEE_t);

    bool    sendCmdBabbling();
    bool    sendCmdLearningPredict();
    bool    sendCmdLearningMove(const int &id, const std::string &partName);

    std::vector<int> reprTaxelsForearm_sim =
    {3, 15, 27, 39, 51, 75, 87, 183,
    207, 255, 291, 303, 315, 339, 351}; //23 taxels

    std::vector<int> reprTaxelsHandL =
    {3, 15, 27, 39, 51, 99, 101, 109, 122, 134};

    std::vector<int> reprTaxelsHandR =
    {3, 15, 39, 51, 101, 118, 137};

public:
    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    attach(yarp::os::RpcServer &source);
    double  getPeriod();
    bool    updateModule();
//    reachBallDemo();


    //Thrift
    bool home_arms()
    {
        bool ok = false;
//        ok = moveArmToStartPos("left");
        ok = moveArmToStartPos("right");
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

    bool reach(bool _useCalib)
    {
        yDebug("[%s] Start reaching the target",name.c_str());
        hasCommand = true;
//        if (hasTarget)
//            return reachTarget();
//        else
//        {
//            yWarning("[%s] No target", name.c_str());
//            return false;
//        }
    }

    bool babble_then_reach(int32_t _id, const std::string &_part)
    {
        bool ok = sendCmdBabbling();
        yarp::os::Time::delay(3);
        ok = ok & sendCmdLearningPredict();
        yarp::os::Time::delay(1);

        if (_id==-1)    // reach all taxel of part
        {
//            if (_part=="b" or _part=="both")
//            {
//                for (int i=0; i<=reprTaxelsHandR.size(); i++)
//                {
//                    sendCmdLearningMove(reprTaxelsHandR[i], "h");

//                }
//            }
//            else if (_part=="h" or _part=="hand")
//            {

//            }
//            else if (_part=="a" or _part=="arm")
//            {

//            }
        }
        else
            ok = ok & sendCmdLearningMove(_id,_part);
        return ok;
    }

    bool set_tilt_angle(const double _ang)
    {
        tiltAngle = _ang;
        return true;
    }

    double get_tilt_angle()
    {
        return tiltAngle;
    }

    bool set_yaw_angle(const double _ang)
    {
        yawAngle = _ang;
        return true;
    }

    double get_yaw_angle()
    {
        return yawAngle;
    }

    bool enable_hand_angle()
    {
        useHandAngle = true;
        return true;
    }

    bool disable_hand_angle()
    {
        useHandAngle = false;
        return true;
    }

};

#endif // REACHBALLDEMO_H
