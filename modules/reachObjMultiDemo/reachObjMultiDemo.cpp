#include "reachObjMultiDemo.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

bool reachObjMultiDemo::configure(yarp::os::ResourceFinder &rf) {
    bool bEveryThingisGood = true;

    name = rf.check("name", Value("reachBallDemo"), "module name (string)").asString();
    period=rf.check("period",Value(0.0)).asDouble();    // as default, update module as soon as receiving new parts from skeleton2D
    changeTipFrame = rf.check("change_tip_frame",Value(1)).asBool();
    useFakeTarget = rf.check("use_fake_target",Value(0)).asBool();
    useHandAngle = rf.check("use_hand_angle",Value(0)).asBool();
    tiltAngle=rf.check("tiltAngle",Value(0.0)).asDouble();
    yawAngle=rf.check("yawAngle",Value(0.0)).asDouble();

    targetDumpedData.open(("/"+name+"/targetDumpedData:o").c_str());

    rpcToMotorBabbling.open(("/"+name+"/motorBabbling/rpc").c_str());
    rpcToLearning.open(("/"+name+"/multisensoryLearning/rpc").c_str());
    if (yarp::os::Network::connect(("/"+name+"/motorBabbling/rpc").c_str(),"/motorBabbling/rpc"))
        yDebug("rpc port to motorBabbling connected!");
    else
        yError("rpc port to motorBabbling cannot connect!");
    if (yarp::os::Network::connect(("/"+name+"/multisensoryLearning/rpc").c_str(),"/multisensoryLearning/rpc"))
        yDebug("rpc port to multisensoryLearning connected!");
    else
        yError("rpc port to multisensoryLearning cannot connect!");


    robot = rf.check("robot", Value("icub")).asString();

    Bottle &start_pos = rf.findGroup("start_position");
    Bottle *b_start_commandHead = start_pos.find("head").asList();
    Bottle *b_start_command = start_pos.find("arm_away").asList();
    Bottle *b_away_command = start_pos.find("arm_away").asList();

    start_command_head.resize(3, 0.0);
    new_command_arm.resize(16, 0.0);
    new_command_head.resize(3, 0.0);
    if ((b_start_commandHead->isNull()) || (b_start_commandHead->size() < 3)) {
        yWarning("[%s] Something is wrong in ini file. Default value is used",name.c_str());
        start_command_head[0] = -20.0;  // -30 in reachBallDemo.ini ==> azimuth angle <-55.0 55.0>  // old -25
        start_command_head[1] = -20.0;  // -20 in reachBallDemo.ini ==> elevation angle <-40.0 30.0>    //old -25
        start_command_head[2] = +10.0;  // ==> vergence angle
    } else {
        for (int i = 0; i < b_start_commandHead->size(); i++) {
            start_command_head[i] = b_start_commandHead->get(i).asDouble();
            yDebug() << start_command_head[i];
        }
    }

    if ((b_start_command->isNull()) || (b_start_command->size() < 16))
    {
        yWarning("[%s] Something is wrong in ini file. Default values are used",name.c_str());
        start_command_arm[0] = -55.0;   start_command_arm[1] =  35.0;   start_command_arm[2] =  0.0;
        start_command_arm[3] =  50.0;   start_command_arm[4] = -45.0;   start_command_arm[5] =  0.0;
        start_command_arm[6] =   0.0;   start_command_arm[7] =  55.0;   start_command_arm[8] = 10.0;
        start_command_arm[9] =  70.0;   start_command_arm[10] =  0.0;   start_command_arm[11] = 0.0;
        start_command_arm[12] =  0.0;   start_command_arm[13] = 70.0;   start_command_arm[14] =120.0;
        start_command_arm[15] =220.0;
    }
    else
    {
        for (int i = 0; i < b_start_command->size(); i++)
            start_command_arm[i] = b_start_command->get(i).asDouble();
    }

    if ((b_away_command->isNull()) || (b_away_command->size() < 16))
    {
        yWarning("[%s] Something is wrong in ini file. Default values are used",name.c_str());
        away_command_arm[0] = -55.0;   away_command_arm[1] = 130.0;   away_command_arm[2] =  0.0;
        away_command_arm[3] =  30.0;   away_command_arm[4] = -45.0;   away_command_arm[5] =  0.0;
        away_command_arm[6] =   0.0;   away_command_arm[7] =  55.0;   away_command_arm[8] = 10.0;
        away_command_arm[9] =   0.0;   away_command_arm[10] =  0.0;   away_command_arm[11] = 0.0;
        away_command_arm[12] =  0.0;   away_command_arm[13] =  0.0;   away_command_arm[14] = 0.0;
        away_command_arm[15] =  0.0;
    }
    else
    {
        for (int i = 0; i < b_away_command->size(); i++)
            away_command_arm[i] = b_away_command->get(i).asDouble();
    }

    setName(name.c_str());

    // Open handler port
    if (!rpcPort.open("/" + getName() + "/rpc"))
    {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/rpc";
        bEveryThingisGood = false;
    }

    // Initialize iCub
    yInfo("[%s] Going to initialise iCub ...",name.c_str());
    while (!initRobot())
        yDebug() << getName() << ": initialising iCub... please wait... ";

    yDebug("[%s] End configuration...",name.c_str());

    attach(rpcPort);
    hasCommand = false;

    // cmdJoints port
    string cmdJointsPort_name = "/"+name+"/cmd_joints:i";
    cmdJointsPort.open(cmdJointsPort_name.c_str());
    yarp::os::Network::connect("/multisensoryLearning/cmd_joints:o",cmdJointsPort_name.c_str());

    string objPoseDimPort_name = "/"+name+"/objPosDim:i";
    objPosDimPort.open(objPoseDimPort_name.c_str());
    yarp::os::Network::connect("/motorBabbling/objPoseDim:o",objPoseDimPort_name.c_str());
    objPoseDim.resize(6,0.0);

    string tactilePort_name = "/"+name+"/skin_events_aggreg:i";
    tactilePort.open(tactilePort_name.c_str());
    yarp::os::Network::connect("/skinEventsAggregator/skin_events_aggreg:o",tactilePort_name.c_str());

    srand (static_cast <unsigned> (time(0)));   // for random value of head

    fakeTarget.resize(3,0.1);
    fakeTarget[0] = -.3;




    return bEveryThingisGood;
}

bool    reachObjMultiDemo::interruptModule()
{
    rpcPort.interrupt();
    cmdJointsPort.interrupt();
    yInfo() << "Bye!";
    return true;
}

bool    reachObjMultiDemo::close()
{
    yInfo() << "Closing module, please wait ... ";

    rightArmDev.close();

    torsoDev.close();

    yDebug("Closing gaze controller..");
    igaze -> stopControl();
    igaze -> restoreContext(contextGaze);
    headDev.close();

    rpcPort.close();

    cmdJointsPort.close();

    yInfo() << "Bye!";

    return true;
}

bool    reachObjMultiDemo::updateModule()
{
    ts.update();
    cmdJointsAng.resize(10,0.0);
    // read the cmdJointsPort port --> obtain the arm-chain angles in joint space
    bool isEvent = false;
    hasTarget = false;
    if (cmdJointsBottle = cmdJointsPort.read(false))
    {
        if (cmdJointsBottle->size()>=12)
        {
            for (int i=0; i<cmdJointsBottle->size()-2; i++)
            {
                cmdJointsAng[i] = cmdJointsBottle->get(i).asDouble();
            }
            isEvent=true;
            hasTarget = true;
            yInfo("target is %s",cmdJointsAng.toString(3,3).c_str());
            yInfo("cmd part: %s",cmdJointsBottle->get(10).asString().c_str());
            yInfo("cmd id: %d",cmdJointsBottle->get(11).asInt());
        }
    }


    if (objPoseDimBottle = objPosDimPort.read(false))
    {
        if (objPoseDimBottle->size()==6)
        {
            for (int i=0; i<objPoseDimBottle->size();i++)
                objPoseDim[i] = objPoseDimBottle->get(i).asDouble();
        }
    }

    Matrix R(3,3);
    // pose x-axis  y-axis       z-axis: palm inward, pointing down
    R(0,0)= 0.0;    R(0,1)= 1.0; R(0,2)= 0.0; // x-coordinate
    R(1,0)= 0.0;    R(1,1)= 0.0; R(1,2)=-1.0; // y-coordinate
    R(2,0)=-1.0;    R(2,1)= 0.0; R(2,2)= 0.0; // z-coordinate

    // pose x-axis y-axis z-axis: palm inward, pointing forward
//    R(0,0)=-1.0; R(0,1)= 0.0; R(0,2)= 0.0; // x-coordinate
//    R(1,0)= 0.0; R(1,1)= 0.0; R(1,2)=-1.0; // y-coordinate
//    R(2,0)= 0.0; R(2,1)=-1.0; R(2,2)= 0.0; // z-coordinate

    Vector handRot0(4,0.0);
    handRot0[1] = 1.0; handRot0[3] = tiltAngle*M_PI/180.0; //Tilt hand 'handAngle' degrees
    Matrix R0 = axis2dcm(handRot0);

    Vector handRot1(4,0.0);
    handRot1[2] = 1.0; handRot1[3] = yawAngle*M_PI/180.0; //Rotate hand 30 degrees of yaw
    Matrix R1 = axis2dcm(handRot1);

    if (useHandAngle)
        R=R*R0.submatrix(0,2,0,2)*R1.submatrix(0,2,0,2);

    hasCommand = true;
    // reach the ball with position direct control mode
    if (hasTarget && hasCommand)
        if (reachTarget())
            yInfo("[%s] can reach target!!!", name.c_str());
    else if (!hasTarget)
    {
        yWarning("[%s] No target", name.c_str());
    }

    return true;
}

double  reachObjMultiDemo::getPeriod()
{
    return period;
}

bool    reachObjMultiDemo::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

bool    reachObjMultiDemo::initRobot()
{
    if(!init_right_arm())
        return false;

    yInfo("[%s] Arms initialized.",name.c_str());

    /* Init. head */
    Property option_head;
    option_head.clear();
    option_head.put("device", "gazecontrollerclient");
    option_head.put("remote", "/iKinGazeCtrl");
    option_head.put("local", ("/"+ name+"/gaze").c_str());

    if (!headDev.open(option_head)) {
        yError("[%s] Device not available.  Here are the known devices:",name.c_str());
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    headDev.view(igaze);
    yInfo("[%s] Head initialized.",name.c_str());

    igaze -> storeContext(&contextGaze);
    igaze -> setSaccadesMode(false);
    igaze -> setNeckTrajTime(0.75);
    igaze -> setEyesTrajTime(0.5);

    igaze->getNeckYawRange(&neck_range_min[0], &neck_range_max[0]);     // yaw ~ azimuth
    igaze->getNeckPitchRange(&neck_range_min[1], &neck_range_max[1]);   // pitch ~ elevation
    igaze->getNeckRollRange(&neck_range_min[2], &neck_range_max[2]);

    for (int8_t i=0; i<3; i++)
        yInfo("[%s] neck range of joint %d: [%3.3f, %3.3f]",
              name.c_str(), i, neck_range_min[i], neck_range_max[i]);

    // Torso controller
    yarp::os::Property OptT;
    OptT.put("robot",  robot);
    OptT.put("part",   "torso");
    OptT.put("device", "remote_controlboard");
    OptT.put("remote", "/"+robot+"/torso");
    OptT.put("local",  "/"+name +"/torso");
    if (!torsoDev.open(OptT))
    {
        yError("[%s]Could not open torso PolyDriver!",name.c_str());
        return false;
    }

    bool okT = 1;

    if (torsoDev.isValid())
    {
        okT = okT && torsoDev.view(encsTorso);
        okT = okT && torsoDev.view(velTorso);
        okT = okT && torsoDev.view(posTorso);
        okT = okT && torsoDev.view(iCtrlTorso);
        okT = okT && torsoDev.view(iCtrlLimTorso);

    }
    int jntsT;
    encsTorso->getAxes(&jntsT);
    encodersTorso.resize(jntsT);

    if (!okT)
    {
        yError("[%s]Problems acquiring torso interfaces!!!!",name.c_str());
        return false;
    }


    /* Init. cartesian controller for right arm */
    Property optionR("(device cartesiancontrollerclient)");
    optionR.put("remote",("/"+robot+"/cartesianController/right_arm").c_str());
    optionR.put("local",("/"+name+"/cart_ctrl/right_arm").c_str());

    if (!rightCartDev.open(optionR)) {
        yError("[%s] Right Arm Cartesian device not available.  Here are the known devices:",name.c_str());
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    rightCartDev.view(iCartCtrlR);
    yInfo("[%s] Right arm Cartesian device initialized.",name.c_str());
    yInfo("[%s] > Initialisation done.",name.c_str());


    if (changeTipFrame)
    {
        Vector joints;
        iCub::iKin::iCubFinger finger("right_index");                  // relevant code to get the position of the finger tip
        finger.getChainJoints(encodersRightArm,joints);                // wrt the end-effector frame
        Matrix tipFrame=finger.getH((M_PI/180.0)*joints);
        Vector tip_x=tipFrame.getCol(3);
        Vector tip_o=yarp::math::dcm2axis(tipFrame);
        iCartCtrlR->attachTipFrame(tip_x,tip_o);                // establish the new controlled frame
    }

    /* iKinChain*/
    arm = new iCub::iKin::iCubArm("right");
    qA.resize(7);

    return true;
}

bool    reachObjMultiDemo::moveHeadToCentralPos()
{
    Vector ang = start_command_head;
    Vector angCur(3,0.0), tmp(3,0.0);
    bool ok = false;
    double dist = 10.0;
    ang[0] = 0.0;
    tmp[0] = 10.0;

    while (dist >= 0.5 || fabs(tmp[0])>= 0.5)
    {
        ok = igaze->lookAtAbsAngles(ang);
        yarp::os::Time::delay(0.1);
        igaze->getAngles(angCur);          // get the current angular configuration

        for (int8_t i=0;i<tmp.size();i++)
            tmp[i] = ang[i]-angCur[i];
        dist = yarp::math::norm(tmp);

        yInfo("[%s] moveHeadToCentralPos: ang = [%s], angCur = [%s]",
              name.c_str(),ang.toString(3,3).c_str(),angCur.toString(3,3).c_str());
    }

    yDebug("[%s] Angle distance = %f",name.c_str(),dist);

    return ok;
}


bool    reachObjMultiDemo::moveArmAway(const string &partName)
{
    Vector command(16,0.0);
    if (partName == "left")
    {

    }
    else if (partName == "right")
    {
        command = encodersRightArm;
        iCtrlArm = iCtrlRightArm;
        posArm = posRightArm;
    }
    else
        return false;

    for (int i = 0; i < 16; i++)
    {
        iCtrlArm->setControlMode(i, VOCAB_CM_POSITION);
        command[i] = away_command_arm[i];
    }
    yInfo("[%s] final joint command with away hand joints: %s",name.c_str(), command.toString(3,3).c_str());
    return posArm->positionMove(command.data());
}


bool    reachObjMultiDemo::moveArmToStartPos(const string &partName)
{
    Vector command; // Command after correction
    command.resize(16);

    if (partName == "left")
    {

    }
    else if (partName == "right")
    {
        command = encodersRightArm;
        iCtrlArm = iCtrlRightArm;
        posArm = posRightArm;
        iCartCtrl = iCartCtrlR;
    }
    else
        return false;

    Matrix R(3,3);
    // pose x-axis  y-axis       z-axis
    R(0,0)=-1.0;    R(0,1)= 0.0; R(0,2)= 0.0; // x-coordinate
    R(1,0)= 0.0;    R(1,1)=-1.0; R(1,2)= 0.0; // y-coordinate
    R(2,0)= 0.0;    R(2,1)= 0.0; R(2,2)= 1.0; // z-coordinate


    if(partName == "left")
    {
        R(1,1)=1.0;
    }

    for (int i = 0; i < 16; i++)
    {
        iCtrlArm->setControlMode(i, VOCAB_CM_POSITION);
        command[i] = start_command_arm[i];
    }
    yInfo("[%s] final joint command with init hand joints: %s",name.c_str(), new_command_arm.toString(3,3).c_str());

    posArm->positionMove(command.data());
    Vector commandTorso(3,0.0);
    posTorso->positionMove(commandTorso.data());

    return true;
}

bool    reachObjMultiDemo::gotoStartPos(bool moveAway=false)
{
    igaze->stopControl();
    velRightArm->stop();

    yarp::os::Time::delay(2.0);

    /* Move arm to start position */
    if (part == "left" || part == "right")
    {
        if (moveAway)
            moveArmAway(part);
        else
            moveArmToStartPos(part);
        bool done_arm = false;
        while (!done_arm)
        {
            posRightArm->checkMotionDone(&done_arm);

            Time::delay(0.04);
        }
        yInfo() << "Done.";

        Time::delay(1.0);
    }
    else
    {
        yError("[%s] Don't know which part to move to start position.",name.c_str());
        return false;
    }
    return true;
}

bool    reachObjMultiDemo::init_right_arm()
{
    /* Create PolyDriver for right arm */
    Property option_right;

    string portnameRightArm = "right_arm";
    option_right.put("robot", robot.c_str());
    option_right.put("device", "remote_controlboard");
    Value &robotnameRightArm = option_right.find("robot");

    option_right.put("local", "/" +name + "/" + robotnameRightArm.asString() + "/" + portnameRightArm + "/control");
    option_right.put("remote", "/" + robotnameRightArm.asString() +  "/" + portnameRightArm);

    yDebug() << "option right arm: " << option_right.toString().c_str();

    if (!rightArmDev.open(option_right))
    {
        yError() << "Device not available.  Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    rightArmDev.view(posRightArm);
    rightArmDev.view(velRightArm);
    rightArmDev.view(itrqRightArm);
    rightArmDev.view(encsRightArm);
    rightArmDev.view(iCtrlRightArm);
    rightArmDev.view(iCtrlLimRightArm);

    joints_arm_max.resize(16);
    joints_arm_min.resize(16);
    for (int l = 0; l < 16; l++)
    {
        iCtrlLimRightArm->getLimits(l, &minLimArm[l], &maxLimArm[l]);
        joints_arm_min[l] = minLimArm[l];
        joints_arm_max[l] = maxLimArm[l];
    }
    yInfo("joints_arm_min: %s", joints_arm_min.subVector(0,6).toString(3,3).c_str());
    yInfo("joints_arm_max: %s", joints_arm_max.subVector(0,6).toString(3,3).c_str());

    for (int l = 0; l < 16; l++)
        yInfo() << "Joint " << l << ": limits = [" << minLimArm[l] << ","
                << maxLimArm[l] << "]. start_commad = " << start_command_arm[l];

    if (posRightArm == nullptr || encsRightArm == nullptr || velRightArm == nullptr ||
            itrqRightArm == nullptr || iCtrlRightArm == nullptr || encsRightArm == nullptr)
    {
        yError() << "Cannot get interface to robot device for right arm";
        rightArmDev.close();
        return false;
    }

    int nj_right = 0;
    posRightArm->getAxes(&nj_right);
    encodersRightArm.resize(nj_right);

    yInfo() << "Wait for arm encoders";
    while (!encsRightArm->getEncoders(encodersRightArm.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for arm encoders";
    }

    return true;
}

bool    reachObjMultiDemo::reachTarget()
{
    hasCommand = false;
    tactilePort.read(false);    // dumbed read to clean the buffer
    Vector target = cmdJointsAng;   // torso + arm
//    Vector calibTarget(7,0.0), target_origin = target;
    yDebug("[%s] Target is: %s", name.c_str(), target.toString(3,3).c_str());


    // reaching here
    Vector velArm(7), velArmHand(16), targetArmHand(16), targetTorso(3), velT(3);
    velArmHand = 20.0;
    velArm = 10.0;
    velT = 20.0;
    velArmHand.setSubvector(0,velArm);

    for (int i = 7; i < 16; i++)
    {
        targetArmHand[i] = start_command_arm[i];
    }

    Vector xEE_t(3,0.0);
    updateArmChain(target, xEE_t);

    targetArmHand.setSubvector(0,target.subVector(3,9));
    posRightArm->setRefSpeeds(velArmHand.data());
    posRightArm->positionMove(targetArmHand.data());

    targetTorso = target.subVector(0,2);
    posTorso->setRefSpeeds(velT.data());
    posTorso->positionMove(targetTorso.data());

    yDebug("targetArmHand: %s",targetArmHand.toString(3,3).c_str());
    yDebug("targetTorso  : %s",targetTorso.toString(3,3).c_str());

    Bottle& output = targetDumpedData.prepare();
    Bottle temp;
    output.clear();

    bool doneArm=false, doneTorso=false;
    double t0=yarp::os::Time::now();
    while ((!doneArm || !doneTorso) && (yarp::os::Time::now()-t0<10.0))
    {
        posRightArm->checkMotionDone(&doneArm);
        posTorso->checkMotionDone(&doneTorso);
        yarp::os::Time::delay(0.1);
    }

    encsRightArm->getEncoders(encodersRightArm.data());
    encsTorso->getEncoders(encodersTorso.data());

    yDebug("final ArmHand: %s",encodersRightArm.toString(3,3).c_str());
    yDebug("final Torso  : %s",encodersTorso.toString(3,3).c_str());

    // Object position and dim and add to output
    for (int i=0; i<objPoseDim.size(); i++)
        output.addDouble(objPoseDim[i]);

    // End-effector position added to ouput
    Vector x_cur(3,0.0), o_cur(4,0.0);
    iCartCtrlR->getPose(x_cur,o_cur);
    for (int i=0; i<x_cur.size(); i++)
        output.addDouble(x_cur[i]);

    // commanded id to reach
    output.addInt(cmdJointsBottle->get(11).asInt());

    Bottle *bArm, *bHand;
    int nb_padding_values = 3;
    if (tactileBottle = tactilePort.read(false))
    {
        yInfo("tactileBottle size: %lu",tactileBottle->size());

        Vector contactArm, contactHand;
        bHand = tactileBottle->get(33).asList();
        bArm = tactileBottle->get(34).asList();
        if (bArm->size()>0)
        {
            for (int i=2; i<bArm->size(); i++)
            {
                contactArm.push_back(bArm->get(i).asInt());
            }
        }

        if (bHand->size()>0)
        {
            for (int i=2; i<bHand->size(); i++)
            {
                contactHand.push_back(bHand->get(i).asInt());
            }
        }
        yDebug("contactArm  = %s",contactArm.toString(3).c_str());
        yDebug("contactHand = %s",contactHand.toString(3).c_str());

        bool isReachedPreciselyA = false, isReachedPreciselyH = false;
//    }


//    if (!tactileBottle->isNull())
//    {
//        if (cmdJointsBottle->get(10).asString()=="a" || cmdJointsBottle->get(10).asString()=="arm" && bArm->size()>0)
        if (bArm->size()>0)
        {
            output.addString(bArm->get(1).asString());
            for (int i=2; i<bArm->size(); i++)
            {
                output.addInt(bArm->get(i).asInt());
                if ((cmdJointsBottle->get(10).asString()=="a" || cmdJointsBottle->get(10).asString()=="arm") &&
                        (cmdJointsBottle->get(11).asInt() == bArm->get(i).asInt()))
                    isReachedPreciselyA = true;
            }
            if ((nb_padding_values-(bArm->size()-2))>0)
                for (int i=0; i<nb_padding_values-(bArm->size()-2); i++)
                    output.addInt(0);
        }
        else
        {
            output.addString("r_forearm");
            for (int i=0; i<nb_padding_values; i++)
                output.addInt(0);   // add dumb value to ease the post-processing
        }
//        else if (cmdJointsBottle->get(10).asString()=="h" || cmdJointsBottle->get(10).asString()=="hand" && bHand->size()>0)
        if (bHand->size()>0)
        {
            output.addString(bHand->get(1).asString());
            for (int i=2; i<bHand->size(); i++)
            {
                output.addInt(bHand->get(i).asInt());
                if ((cmdJointsBottle->get(10).asString()=="h" || cmdJointsBottle->get(10).asString()=="hand") &&
                        (cmdJointsBottle->get(11).asInt() == bHand->get(i).asInt()))
                    isReachedPreciselyH = true;
            }
            if ((nb_padding_values-(bHand->size()-2))>0)
                for (int i=0; i<nb_padding_values-(bHand->size()-2); i++)
                    output.addInt(0);
        }
        else
        {
            output.addString("r_hand");
            for (int i=0; i<nb_padding_values; i++)
                output.addInt(0);   // add dumb value to ease the post-processing
        }
        if (isReachedPreciselyA)
            output.addString("True");
        else
            output.addString("False");
        if (isReachedPreciselyH)
            output.addString("True");
        else
            output.addString("False");
    }
    else
    {
        output.addString("r_forearm");
        for (int i=0; i<nb_padding_values; i++)
            output.addInt(0);   // add dumb value to ease the post-processing
        output.addString("r_hand");
        for (int i=0; i<nb_padding_values; i++)
            output.addInt(0);   // add dumb value to ease the post-processing
        output.addString("False");
        output.addString("False");
    }

    posRightArm->stop();
    posTorso->stop();

    targetDumpedData.setEnvelope(ts);
    targetDumpedData.write();

    return true;
}


void    reachObjMultiDemo::updateArmChain(const Vector &q, Vector& xEE_t)
{
//    encsRightArm->getEncoders(encodersRightArm->data());
//    qA=encodersRightArm;
    qA = q;

    arm->setAng(qA*iCub::ctrl::CTRL_DEG2RAD);
    //H=arm->getH();
    //x_t=H.subcol(0,3,3);
    xEE_t = arm->EndEffPosition();
    Vector o_t = arm->EndEffPose().subVector(3,5);
}

bool    reachObjMultiDemo::sendCmdBabbling()
{
    Bottle cmd, rep;
    cmd.addString("auto_obj_only");
    cmd.addString("right");
    cmd.addInt(1);

    if (rpcToMotorBabbling.write(cmd, rep))
        return rep.get(0).asBool();
    else
        return false;
}

bool    reachObjMultiDemo::sendCmdLearningPredict()
{
    Bottle cmd, rep;
    cmd.addString("predict");

    if (rpcToLearning.write(cmd, rep))
        return true;
    else
        return false;
}

bool    reachObjMultiDemo::sendCmdLearningMove(const int &id, const string &partName)
{
    Bottle cmd, rep;
    cmd.addString("move");
    cmd.addInt(id);
    cmd.addString(partName);

    if (rpcToLearning.write(cmd, rep))
        return true;
    else
        return false;
}

