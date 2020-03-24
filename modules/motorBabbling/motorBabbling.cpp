#include "motorBabbling.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

// TODO:
// - add mode babbling-with-object
// - add number of time babbling-with-object
// - generate objects randomly (1) position, (2) size, (3) color, (4) shape. For each object, do following N times:
//  + look at object with random head configuration
//  + capture stereo-images: read images and dump once
//  + move arm randomly toward object, stop if collision happen
//  + collect data: head encoders, arm encoders, tactile (activation taxels, skin part)-read from "skinEventAggregation" (local change version)


double genRandom()
{
    return static_cast <double> (rand()) / static_cast <double> (RAND_MAX);    // random double number in <0.0 1.0>
}

bool motorBabbling::configure(yarp::os::ResourceFinder &rf) {
    bool bEveryThingisGood = true;

    name = rf.check("name", Value("motorBabbling"), "module name (string)").asString();
    period=rf.check("period",Value(0.0)).asDouble();    // as default, update module as soon as receiving new parts from skeleton2D

    robot = rf.check("robot", Value("icub")).asString();
    single_joint = rf.check("single_joint", Value(-1)).asInt();

    eePoseOutPort.open(("/"+name+"/eePose:o").c_str());
    objPoseDimOutPort.open(("/"+name+"/objPoseDim:o").c_str());

    Bottle &start_pos = rf.findGroup("start_position");
    Bottle *b_start_commandHead = start_pos.find("head").asList();
    Bottle *b_start_command = start_pos.find("arm").asList();
    Bottle *b_away_command = start_pos.find("arm_away").asList();

    start_command_head.resize(3, 0.0);
    new_command_arm.resize(16, 0.0);
    new_command_head.resize(3, 0.0);
    if ((b_start_commandHead->isNull()) || (b_start_commandHead->size() < 3)) {
        yWarning("[%s] Something is wrong in ini file. Default value is used",name.c_str());
        start_command_head[0] = 20.0;  // -30 in motorBabbling.ini ==> azimuth angle <-55.0 55.0>  // old -25
        start_command_head[1] = -20.0;  // -20 in motorBabbling.ini ==> elevation angle <-40.0 30.0>    //old -25
        start_command_head[2] = +10.0;  // ==> vergence angle
    } else {
        for (int i = 0; i < b_start_commandHead->size(); i++) {
            start_command_head[i] = b_start_commandHead->get(i).asDouble();
            yDebug() << start_command_head[i];
        }
    }

    fixatePos.resize(3,0.0);
    curRepeat = 0;
    nbRepeat = 0;
    isBabbling = false;
    moveHeadOnly = false;

    // babbling with object mode
    nbRepeatObjGen = 0;
    babbling_with_obj = false;

    generateObjRandom_only = false;

    // camera port
    imagePortInR = new BufferedPort<ImageOf<PixelRgb>>;
    imagePortInL = new BufferedPort<ImageOf<PixelRgb>>;

    imagePortInR->open(("/"+name+"/imageR:i").c_str());
    imagePortInL->open(("/"+name+"/imageL:i").c_str());
    imagePortOutR.open(("/"+name+"/imageR:o").c_str());
    imagePortOutL.open(("/"+name+"/imageL:o").c_str());


    if (robot=="icubSim")
    {
        if (yarp::os::Network::connect("/icubSim/cam/left",imagePortInL->getName().c_str()))
            yDebug("left image connected!");
        else
            yError("left image cannot connect!");
        if (yarp::os::Network::connect("/icubSim/cam/right",imagePortInR->getName().c_str()))
            yDebug("right image connected!");
        else
            yError("right image cannot connect!");

        string port2icubsim = "/" + name + "/sim:o";
        //cout<<port2icubsim<<endl;
        if (!portToSimWorld.open(port2icubsim.c_str())) {
            yError("Unable to open port << port2icubsim << endl");
        }
        std::string port2world = "/icubSim/world";
        yarp::os::Network::connect(port2icubsim, port2world.c_str());

        Time::delay(0.5);

        Matrix T_world_root(4,4);

        T_world_root(0,1)=-1;
        T_world_root(1,2)=1; T_world_root(1,3)=0.5976;
        T_world_root(2,0)=-1; T_world_root(2,3)=-0.026;
        T_world_root(3,3)=1;
        T_root_world=yarp::math::SE3inv(T_world_root);

        cleanWorld();
    }
    else
    {
        if (yarp::os::Network::connect("/icub/camcalib/left/out",imagePortInL->getName().c_str()))
            yDebug("left image connected!");
        else
            yError("left image cannot connect!");
        if (yarp::os::Network::connect("/icub/camcalib/right/out",imagePortInR->getName().c_str()))
            yDebug("right image connected!");
        else
            yError("right image cannot connect!");
    }
    genObjPos.resize(3,0.0);

    // head encoders
    headEncPortOut.open(("/"+name+"/headEncs:o").c_str());


    if ((b_start_command->isNull()) || (b_start_command->size() < 16))
    {
        yWarning("[%s] Something is wrong in ini file. Default values are used",name.c_str());
        start_command_arm[0] = -55.0;   start_command_arm[1] =  35.0;   start_command_arm[2] =  0.0;
        start_command_arm[3] =  50.0;   start_command_arm[4] = -45.0;   start_command_arm[5] =  0.0;
        start_command_arm[6] =   0.0;   start_command_arm[7] =  55.0;   start_command_arm[8] = 10.0;
        start_command_arm[9] =   0.0;   start_command_arm[10] =  0.0;   start_command_arm[11] = 0.0;
        start_command_arm[12] =  0.0;   start_command_arm[13] =  0.0;   start_command_arm[14] = 0.0;
        start_command_arm[15] =  0.0;
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

    Bottle &babbl_par = rf.findGroup("babbling_param");
    freq = babbl_par.check("freq", Value(0.2)).asDouble();
    amp = babbl_par.check("amp", Value(5.0)).asDouble();
    duration = babbl_par.check("duration", Value(3.0)).asDouble();
    yDebug("[%s] duration = %f", name.c_str(), duration);

    if (robot=="icub")
        ampH = 5.0;
    else if (robot == "icubSim")
        ampH = 10.0;

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

    srand (static_cast <unsigned> (time(0)));   // for random value of head

    return bEveryThingisGood;
}

bool    motorBabbling::interruptModule()
{
    rpcPort.interrupt();
    imagePortInL->interrupt();
    imagePortInR->interrupt();
    imagePortOutL.interrupt();
    imagePortOutR.interrupt();
    yInfo() << "Bye!";
    return true;
}

bool    motorBabbling::close()
{
    yInfo() << "Closing module, please wait ... ";

    leftArmDev.close();
    rightArmDev.close();
    headCartDev.close();
    leftArmDev.close();
    rightArmDev.close();

    yInfo() << "close rpcPort";
    rpcPort.close();
    yInfo() << "close input image ports";
    imagePortInL->close();
    imagePortInR->close();

    yInfo() << "close output image ports";
    imagePortOutL.close();
    imagePortOutR.close();

    yInfo() << "delete input image ports";
    delete imagePortInL;
    delete imagePortInR;

    yInfo() << "Bye!";

    return true;
}

bool    motorBabbling::updateModule()
{
    // TODO: use interruptBabbling
    ts.update();
    switch (operation_mode) {
    case OBJ_MODE:
        cleanWorld();
        home_all();
        Time::delay(1.);
        generateObjRandom(autoArmName,genObjPos);

        moveHeadToStartPos(true);
        operation_mode = IDLE_MODE;
        generateObjRandom_only = false;
        posArm->stop();
        posHead->stop();
        break;

//    default:
//        break;
//    }
//    if (generateObjRandom_only)
//    {
////        cleanWorld();
//        home_all();
//        Time::delay(1.);
//        generateObjRandom(autoArmName,genObjPos);
//        generateObjRandom_only = false;
//    }
//    else
//    {
    case MOV_MODE:
//        cleanWorld();
        if (isBabbling) //isBabbling set by sending rpc command "babbling_<part>"
        {
            if (Time::now()< startBabblingTime + duration)
            {
                double t = Time::now() - startBabblingTime;
    //            yInfo() << "t = " << t << "/ " << duration;
                if (!babbling_with_obj)
                    moveHeadRandomly();
                if (!moveHeadOnly)
                    babblingCommands(t, single_joint);  // Comment for testing
                Vector x_cur(3,0.0), o_cur(4,0.0);
                iCartCtrl->getPose(x_cur,o_cur);
    //            yInfo("current pos: %s",x_cur.toString(3,3).c_str());

                Bottle& output = eePoseOutPort.prepare();
                output.clear();
    //            for (int8_t i=0;i<x_cur.size();i++)
    //                output.addDouble(x_cur[i]);
                addVectorToBottle(x_cur,output);
                eePoseOutPort.setEnvelope(ts);
                eePoseOutPort.write();
            }
            else
            {
                yDebug("[%s] ============> babbling with COMMAND is FINISHED",name.c_str());
                isBabbling = false; // isBabbling uset when the time is over the duration
                if (curRepeat>=nbRepeat)
                    moveHeadOnly = false;
                igaze->stopControl();
                velLeftArm->stop();
                velRightArm->stop();
            }
        }
        else
        {
            if(curRepeat<nbRepeat)
            {
                if(moveHeadOnly)
                {
                    if(babble_head(autoArmName))
                    {
                        curRepeat++;
                        moveHeadOnly = true;
                        yInfo("[%s] Finish %d time babbling the head in the %s side",
                            name.c_str(), curRepeat, autoArmName.c_str());
                    }
                    else
                        yError("[%s] Error running %d time babbling the head in the %s side",
                            name.c_str(), curRepeat, autoArmName.c_str());
                }
                else
                {
                    if (babbling_with_obj && curRepeatObjGen<nbRepeatObjGen && curRepeat==0)
                    {
                        home_all();
                        Time::delay(.5);
                        if (generateObjRandom(autoArmName,genObjPos))
                            curRepeatObjGen++;
                    }

                    if(babble_arm(autoArmName))
                    {
                        curRepeat++;
                        yInfo("[%s] Finish %d time babbling the %s arm",
                            name.c_str(), curRepeat, autoArmName.c_str());
                        Vector x_cur(3,0.0), o_cur(4,0.0);
                        iCartCtrl->getPose(x_cur,o_cur);
                        yInfo("current pos: %s",x_cur.toString(3,3).c_str());

                        Bottle& output = eePoseOutPort.prepare();
                        output.clear();
                        for (int8_t i=0;i<x_cur.size();i++)
                            output.addDouble(x_cur[i]);
                        eePoseOutPort.write();
                    }
                    else
                        yError("[%s] Error running %d time babbling the %s arm",
                            name.c_str(), curRepeat, autoArmName.c_str());
                }
            }
            else
            {
                if (!babbling_with_obj || curRepeatObjGen==nbRepeatObjGen)
                {
                    nbRepeat = 0;
                    operation_mode = IDLE_MODE;
                }
                if (babbling_with_obj && curRepeatObjGen==nbRepeatObjGen)
                {
                    babbling_with_obj = false;
                    operation_mode = IDLE_MODE;
                }
                curRepeat = 0;
                cleanWorld();
//                operation_mode = IDLE_MODE;
            }
        }
        break;

    default:
        operation_mode = IDLE_MODE;
        break;
    }
    return true;
}

double  motorBabbling::getPeriod()
{
    return period;
}

bool    motorBabbling::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

bool    motorBabbling::initArm(const std::string &armName, yarp::dev::PolyDriver &armDev)
{
    Property option;
    yarp::sig::Vector encodersArm;

    string portName = armName;  // part;
    option.put("robot", robot.c_str());
    option.put("device", "remote_controlboard");
    Value &robotName = option.find("robot");

    option.put("local", ("/" +name + "/" + robotName.asString() + "/" + portName + "/control").c_str());
    option.put("remote", "/" + robotName.asString() + "/" + portName);

    yDebug() << "option " << armName.c_str() << " arm: " << option.toString().c_str();

    if (!armDev.open(option))
    {
        yError(" [%s] Device not available. Here are the known devices:",name.c_str());
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    if (armName == "left_arm")
    {
        posArm = posLeftArm;
        velArm = velLeftArm;
        itrqArm = itrqLeftArm;
        encsArm = encsLeftArm;
        iCtrlArm = iCtrlLeftArm;
        iCtrlLimArm = iCtrlLimLeftArm;
    }
    else if (armName == "right_arm")
    {
        posArm = posRightArm;
        velArm = velRightArm;
        itrqArm = itrqRightArm;
        encsArm = encsRightArm;
        iCtrlArm = iCtrlRightArm;
        iCtrlLimArm = iCtrlLimRightArm;
    }

    armDev.view(posArm);
    armDev.view(velArm);
    armDev.view(itrqArm);
    armDev.view(encsArm);
    armDev.view(iCtrlArm);
    armDev.view(iCtrlLimArm);

    double minLimArm[16];
    double maxLimArm[16];
    for (int l = 0; l < 16; l++)
        iCtrlLimArm->getLimits(l, &minLimArm[l], &maxLimArm[l]);

    for (int l = 0; l < 16; l++)
        yInfo() << "Joint " << l << ": limits = [" << minLimArm[l] << ","
                << maxLimArm[l] << "]. start_commad = " << start_command_arm[l];

    if (posArm == nullptr || encsArm == nullptr || velArm == nullptr ||
            itrqArm == nullptr || iCtrlArm == nullptr || encsArm == nullptr)
    {
        yError() << "Cannot get interface to robot device for left arm";
        armDev.close();
    }

    int nj = 0;
    posArm->getAxes(&nj);
    if (armName == "left_arm")
    {
        encodersLeftArm.resize(nj);
        encodersArm = encodersLeftArm;
    }
    else if (armName == "right_arm")
    {
        encodersRightArm.resize(nj);
        encodersArm = encodersRightArm;
    }

    yDebug("nj = %d, sz encodersLeftArm = %lu",nj,encodersLeftArm.size());
    yInfo("[%s] Wait for %s arm encoders",name.c_str(), armName.c_str());
    if (armName == "left_arm")
    {
        while (!encsArm->getEncoders(encodersLeftArm.data()))
        {
            Time::delay(0.1);
            yInfo("[%s] Wait for %s arm encoders",name.c_str(), armName.c_str());
        }
    }
    else if (armName == "right_arm")
    {
        while (!encsArm->getEncoders(encodersRightArm.data()))
        {
            Time::delay(0.1);
            yInfo("[%s] Wait for %s arm encoders",name.c_str(), armName.c_str());
        }
    }

    return true;
}

bool    motorBabbling::initRobot()
{
//    if(!initArm("left_arm", leftArmDev))
//        return false;
//    if(!initArm("right_arm", rightArmDev))
//        return false;

    if(!init_left_arm())
        return false;
    if(!init_right_arm())
        return false;

    yInfo("[%s] Arms initialized.",name.c_str());
    if(!init_head())
        return false;

    /* Init. head */
    Property option_head;
    option_head.clear();
    option_head.put("device", "gazecontrollerclient");
    option_head.put("remote", "/iKinGazeCtrl");
    option_head.put("local", ("/"+ name+"/gaze").c_str());

    if (!headCartDev.open(option_head)) {
        yError("[%s] Device not available.  Here are the known devices:",name.c_str());
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    headCartDev.view(igaze);
    yInfo("[%s] Head initialized.",name.c_str());

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

    /* Init. cartesian controller for left arm */
    Property optionL("(device cartesiancontrollerclient)");
    optionL.put("remote",("/"+robot+"/cartesianController/left_arm").c_str());
    optionL.put("local",("/"+name+"/cart_ctrl/left_arm").c_str());

    if (!leftCartDev.open(optionL)) {
        yError("[%s] Left Arm Cartesian device not available.  Here are the known devices:",name.c_str());
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    leftCartDev.view(iCartCtrlL);
    yInfo("[%s] Left arm Cartesian device initialized.",name.c_str());

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

    return true;
}

bool    motorBabbling::moveHeadToStartPos(bool ranPos=true)
{
    Vector ang = start_command_head;
    double r;

    // obtain random angle of neck around object --> this to ensure object is inside the view
    yDebug("genObjPos = %s", genObjPos.toString(3,3).c_str());
    if (babbling_with_obj || generateObjRandom_only)
    {
        Vector tempPos = genObjPos;
        do{
            for (int8_t i=0; i<3;i++)
            {
                tempPos[i] = genObjPos[i] + 0.15*(2.*genRandom() - 1.);
            }
            tempPos[0] = min(-0.2,tempPos[0.0]);
            yDebug("tempPos = %s", tempPos.toString(3,3).c_str());

            igaze->getAnglesFrom3DPoint(tempPos, ang);
            yDebug("ang = %s", ang.toString(3,3).c_str());
        }
        while(ang[0]>55. || (ang[0]<0 && part=="right") || ang[0]<-55. || (ang[0]>0 && part=="left")
              || ang[1]<-55. || ang[1]>30. );
        yInfo("ang = %s", ang.toString(3,3).c_str());
    }
    else
    {
        if (ranPos)
            for (int8_t i=0; i<2;i++)
            {
                r = genRandom();
    //            ang[i] += (20.0-i*5.0)*(2.0*r-1.0);
    //            ang[i] += (30.0-i*5.0)*(2.0*r-1.0); // azimuth <-50 ~ 10>, elevation <-35 ~ 15>
                ang[i] += (start_command_head[i]-neck_range_min[i])*(2.0*r-1.0); // For collecting data
    //            ang[i] += 5.0*(2.0*r-1.0);  // For testing
    //            ang[i] = (5.0-neck_range_min[i])*r;
            }
        if (part == "left")
            ang[0] = -ang[0];
    }

    new_command_head = ang;
    yInfo("[%s] new generated head gazing angles [%s]",name.c_str(),new_command_head.toString(3,3).c_str());

    igaze->get3DPointFromAngles(0, ang, fixatePos);
    yInfo("[%s] corresponding fixated position [%s]",name.c_str(),fixatePos.toString(3,3).c_str());
    return igaze->lookAtAbsAngles(ang);
}

bool    motorBabbling::moveHeadRandomly()
{
    Vector ang = new_command_head;  // new_command_head changed during moveHeadToStartPos
    double r;
    for (int8_t i=0; i<2;i++)
    {
        r = genRandom();
//        double ampH;
        ang[i] += ampH*(2.0*r-1.0);
    }

//    yInfo("[%s] new generated head gazing angles [%s]",name.c_str(),ang.toString(3,3).c_str());
    return igaze->lookAtAbsAngles(ang);
}

bool    motorBabbling::moveHeadToCentralPos()
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

bool    motorBabbling::moveTorsoToHome()
{
    Vector targetTorso(3,0.0);
    posTorso->positionMove(targetTorso.data());
    bool doneTorso=false;
    double t0=yarp::os::Time::now();
    while (!doneTorso && (yarp::os::Time::now()-t0<5.0))
    {
        posTorso->checkMotionDone(&doneTorso);
        yarp::os::Time::delay(0.1);
    }
}

bool    motorBabbling::moveArmAway(const string &partName)
{
    Vector command(16,0.0);
    if (partName == "left")
    {
        command = encodersLeftArm;
        iCtrlArm = iCtrlLeftArm;
        posArm = posLeftArm;
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


bool    motorBabbling::moveArmToStartPos(const string &partName, bool home = false )
{
    Vector command; // Command after correction
    command.resize(16);

    if (partName == "left")
    {
        command = encodersLeftArm;
        iCtrlArm = iCtrlLeftArm;
        posArm = posLeftArm;
        iCartCtrl = iCartCtrlL;

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
    Vector od= yarp::math::dcm2axis(R);
//    Vector od(4,0.0);
    Vector xd, xdhat,odhat, qdhat;
//    if (fixatePos[0]<-0.4) fixatePos[0] = -0.4;

    iCartCtrl->askForPose(fixatePos,od,xdhat,odhat,qdhat);
//    yInfo("[%s] joint command from Cartesian Controller: %s",name.c_str(), qdhat.subVector(3,9).toString(3,3).c_str());
    for (int i = 0; i < 16; i++)
    {
        iCtrlArm->setControlMode(i, VOCAB_CM_POSITION);
        command[i] = start_command_arm[i];
        if (i<=6)
            new_command_arm[i] = qdhat[i+3]+5.0*(2.0*genRandom()-1.0);    // qdhat including 3 torso joints
        else
        {
            new_command_arm[i] = start_command_arm[i];
        }
    }
    yInfo("[%s] final joint command with init hand joints: %s",name.c_str(), new_command_arm.toString(3,3).c_str());
    if (babbling_with_obj || generateObjRandom_only)
        command[1] = 90.;
//    new_command_arm = command;
    Vector velArmHand(16, 20.0);
    posArm->setRefSpeeds(velArmHand.data());
    if (home)
        posArm->positionMove(command.data());
    else
        posArm->positionMove(new_command_arm.data());
    return true;
}

bool    motorBabbling::gotoStartPos(bool moveAway=false)
{
    igaze->stopControl();
    velLeftArm->stop();
    velRightArm->stop();

//    yarp::os::Time::delay(2.0);
    if (babbling_with_obj)
    {
        home_arms();
        Time::delay(1.);
    }
    moveHeadToStartPos();

    if (babbling_with_obj)
    {
        Time::delay(2.);
        // TODO: Take images and head encoders here: --> implicitly 3D information of object
        yDebug("read images");
        imageInR = imagePortInR->read(false);
        imageInL = imagePortInL->read(false);

        imagePortOutR.setEnvelope(ts);
        imagePortOutL.setEnvelope(ts);
        ImageOf<PixelRgb> imgOutR, imgOutL;
        yDebug("copy images");
        if (imageInL!=NULL)
        {
            imgOutR.copy(*imageInR);
            yDebug("copy right images");
        }

        if (imageInL!=NULL)
        {
            imgOutL.copy(*imageInL);
            yDebug("copy left images");
        }

        yDebug("write images");
        imagePortOutR.write(imgOutR);
        imagePortOutL.write(imgOutL);

        while (!encsHead->getEncoders(encodersHead.data()));
        {
            Time::delay(0.05);
        }
        yInfo("[%s] Head encoders: %s",name.c_str(), encodersHead.toString(3,3).c_str());
        Bottle& output = headEncPortOut.prepare();
        output.clear();
        addVectorToBottle(encodersHead,output);
        headEncPortOut.setEnvelope(ts);
        headEncPortOut.write();

    }

    /* Move arm to start position */
    if (part == "left" || part == "right")
    {
        if (moveAway)
            moveArmAway(part);
        else
            moveArmToStartPos(part);
        bool done_head = false;
        bool done_arm = false;
        double t0 = Time::now();
        while (!done_head || !done_arm)
        {
            igaze->checkMotionDone(&done_head);
            if (part == "left")
                posLeftArm->checkMotionDone(&done_arm);
            else
                posRightArm->checkMotionDone(&done_arm);

            if (babbling_with_obj && (Time::now()-t0>5.))
                done_arm = true;
            Time::delay(0.04);
        }
        yInfo() << "Done.";

//        Time::delay(1.0);
    }
    else
    {
        yError("[%s] Don't know which part to move to start position.",name.c_str());
        return false;
    }
    startBabblingTime = yarp::os::Time::now();
    return true;
}

void    motorBabbling::babblingCommands(const double &t, int j_idx)
{
    double ref_command[16]; // Reference command for the 16 arm joints
    yarp::sig::Vector command; // Command after correction
    yarp::sig::Vector encodersUsed;
    command.resize(16);

    for (unsigned int l = 0; l < command.size(); l++)
        command[l] = 0;

    for (unsigned int l = 0; l < 16; l++)
    {
//        ref_command[l] = start_command_arm[l] + amp * sin(freq * t * 2 * M_PI);
        double r1 = genRandom();
        double r2 = genRandom();
        ref_command[l] = new_command_arm[l] + (2.0*r1-1.0)*amp * sin(freq*(2.0*r2-1.0) * t * 2 * M_PI);
//        ref_command[l] = new_command_arm[l] + r1*amp * sin(freq*(2.0*r2-1.0) * t * 2 * M_PI);
        ref_command[l] = std::max(minLimArm[l],ref_command[l]);
        ref_command[l] = std::min(maxLimArm[l],ref_command[l]);
    }

    if (part == "right" || part == "left")
    {
        bool okEncArm = false;

        if (part == "right")
        {
            encodersUsed = encodersRightArm;
            okEncArm = encsRightArm->getEncoders(encodersUsed.data());
        }
        else
        {
            encodersUsed = encodersLeftArm;
            okEncArm = encsLeftArm->getEncoders(encodersUsed.data());
        }

        if (j_idx != -1) // single joint babbling
        {
            if (!okEncArm)
            {
                yError("[%s] Error receiving encoders",name.c_str());
                command[j_idx] = 0;
            }
            else
            {
                yDebug() << "command before correction = " << ref_command[j_idx];
                yDebug() << "current encoders : " << encodersUsed[j_idx];
                command[j_idx] = amp * sin(freq * t * 2 * M_PI);
                yDebug() << "command after correction = " << command[j_idx];
                if (command[j_idx] > 50) command[j_idx] = 50;
                if (command[j_idx] < -50) command[j_idx] = -50;
                yDebug() << "command after saturation = " << command[j_idx];
            }
        }
        else
        {
            int8_t lMin, lMax, vMax, K;
            if (robot=="icubSim")
            {
                vMax = 30;
                K = 15;
            }
            else
            {
                vMax = 20;
                K = 10;
            }
            if (part_babbling == "arm")
            {
                lMin = 0;   lMax = 7;
            }
            else if (part_babbling == "hand")
            {
                lMin = 7;   lMax = command.size();
            }
            else
            {
                yError("[%s] Can't babble the required body part.",name.c_str());
                return;
            }

            if (!okEncArm)
            {
                yError("[%s] Error receiving encoders",name.c_str());
                for (unsigned int l = lMin; l < lMax; l++) command[l] = 0;
            }
            else
            {
                for (unsigned int l = lMin; l < lMax; l++)
                {
                    command[l] = K * (ref_command[l] - encodersUsed[l]);
                    if (command[j_idx] > vMax) command[j_idx] = vMax;
                    if (command[j_idx] < -vMax) command[j_idx] = -vMax;
                }
            }
        }

        if (part == "right") {
            velRightArm->velocityMove(command.data());
        } else if (part == "left") {
            velLeftArm->velocityMove(command.data());
        } else {
            yError("[%s] Don't know which part to move to do babbling.",name.c_str());
            return;
        }

        yarp::os::Time::delay(0.05); // This delay is needed!!!

    } else {
        yError("[%s] Which arm to babble with?",name.c_str());
    }
}

bool    motorBabbling::startBabbling()
{
    bool homeStart = gotoStartPos(); // First go to home position
    if (!homeStart) {
        yError("[%s] I got lost going home!",name.c_str());
        return false;
    }

    // Change to velocity mode
    for (int i = 0; i < 16; i++) {
        if (part == "right") {
            iCtrlRightArm->setControlMode(i, VOCAB_CM_VELOCITY);
        } else if (part == "left") {
            iCtrlLeftArm->setControlMode(i, VOCAB_CM_VELOCITY);
        } else {
            yError("[%s] Don't know which part to move to do babbling.",name.c_str());
            return false;
        }
    }

//    startBabblingTime = yarp::os::Time::now();
    yDebug("[%s] ============> babbling with COMMAND will START",name.c_str());
    yInfo() << "AMP " << amp << "FREQ " << freq;

    isBabbling = true;

    return true;
}

bool    motorBabbling::init_head()
{
    Property option_head;

    string portnameHead = "head";  // part;
    option_head.put("robot", robot.c_str());
    option_head.put("device", "remote_controlboard");
    Value &robotnameHead = option_head.find("robot");

    option_head.put("local", "/" +name + "/" + robotnameHead.asString() + "/" + portnameHead + "/control");
    option_head.put("remote", "/" + robotnameHead.asString() + "/" + portnameHead);

    yDebug() << "option head: " << option_head.toString().c_str();

    if (!headDev.open(option_head)) {
        yError() << "Device not available. Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    headDev.view(posHead);
    headDev.view(encsHead);
    headDev.view(iCtrlHead);
    headDev.view(iCtrlLimHead);

    if (posHead == nullptr || encsHead == nullptr || iCtrlHead == nullptr || encsHead == nullptr)
    {
        yError() << "Cannot get interface to robot device for left arm";
        headDev.close();
        return false;
    }

    int nj_head = 0;
    posHead->getAxes(&nj_head);
    encodersHead.resize(nj_head);

    yInfo() << "Wait for head encoders";
    while (!encsHead->getEncoders(encodersHead.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for arm encoders";
    }

    return true;
}

bool    motorBabbling::init_left_arm()
{
    Property option_left;

    string portnameLeftArm = "left_arm";  // part;
    option_left.put("robot", robot.c_str());
    option_left.put("device", "remote_controlboard");
    Value &robotnameLeftArm = option_left.find("robot");

    option_left.put("local", "/" +name + "/" + robotnameLeftArm.asString() + "/" + portnameLeftArm + "/control");
    option_left.put("remote", "/" + robotnameLeftArm.asString() + "/" + portnameLeftArm);

    yDebug() << "option left arm: " << option_left.toString().c_str();

    if (!leftArmDev.open(option_left)) {
        yError() << "Device not available. Here are the known devices:";
        yError() << yarp::dev::Drivers::factory().toString();
        return false;
    }

    leftArmDev.view(posLeftArm);
    leftArmDev.view(velLeftArm);
    leftArmDev.view(itrqLeftArm);
    leftArmDev.view(encsLeftArm);
    leftArmDev.view(iCtrlLeftArm);
    leftArmDev.view(iCtrlLimLeftArm);

    double minLimArm[16];
    double maxLimArm[16];
    for (int l = 0; l < 16; l++)
        iCtrlLimLeftArm->getLimits(l, &minLimArm[l], &maxLimArm[l]);

    for (int l = 0; l < 16; l++)
        yInfo() << "Joint " << l << ": limits = [" << minLimArm[l] << ","
                << maxLimArm[l] << "]. start_commad = " << start_command_arm[l];

    if (posLeftArm == nullptr || encsLeftArm == nullptr || velLeftArm == nullptr ||
            itrqLeftArm == nullptr || iCtrlLeftArm == nullptr || encsLeftArm == nullptr)
    {
        yError() << "Cannot get interface to robot device for left arm";
        leftArmDev.close();
        return false;
    }

    int nj_left = 0;
    posLeftArm->getAxes(&nj_left);
    encodersLeftArm.resize(nj_left);

    yInfo() << "Wait for arm encoders";
    while (!encsLeftArm->getEncoders(encodersLeftArm.data()))
    {
        Time::delay(0.1);
        yInfo() << "Wait for arm encoders";
    }

    return true;
}

bool    motorBabbling::init_right_arm()
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

//    double minLimArm[16];
//    double maxLimArm[16];
    for (int l = 0; l < 16; l++)
        iCtrlLimRightArm->getLimits(l, &minLimArm[l], &maxLimArm[l]);

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

void motorBabbling::createStaticSphere(const double& radius, const Vector &pos, const string &color) //pos in sim FoR
{
    Bottle cmd;
    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("ssph");
    cmd.addDouble(radius);

    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    // color
    if (color=="red")
    {
        cmd.addInt(1);cmd.addInt(0);cmd.addInt(0);  //red
    }
    else if (color == "green")
    {
        cmd.addInt(0);cmd.addInt(1);cmd.addInt(0);  //green
    }
    else if (color == "blue")
    {
        cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
    }
    else if (color == "purple")
    {
        cmd.addInt(1);cmd.addInt(0);cmd.addInt(1);  //purple
    }
    else if (color == "yellow")
    {
        cmd.addInt(1);cmd.addInt(1);cmd.addInt(0);  //yellow
    }
    else if (color == "magenta")
    {
        cmd.addInt(0);cmd.addInt(1);cmd.addInt(1);  //magenta
    }
    else
    {
        cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
    }

    portToSimWorld.write(cmd);
}

void motorBabbling::createStaticBox(const Vector &dim, const Vector &pos, const string &color) //pos in sim FoR
{
    Bottle cmd;
    cmd.clear();
    cmd.addString("world");
    cmd.addString("mk");
    cmd.addString("sbox");
    cmd.addDouble(dim(0)); cmd.addDouble(dim(1)); cmd.addDouble(dim(2));

    cmd.addDouble(pos(0));
    cmd.addDouble(pos(1));
    cmd.addDouble(pos(2));
    // color
    if (color=="red")
    {
        cmd.addInt(1);cmd.addInt(0);cmd.addInt(0);  //red
    }
    else if (color == "green")
    {
        cmd.addInt(0);cmd.addInt(1);cmd.addInt(0);  //green
    }
    else if (color == "blue")
    {
        cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
    }
    else if (color == "purple")
    {
        cmd.addInt(1);cmd.addInt(0);cmd.addInt(1);  //purple
    }
    else if (color == "yellow")
    {
        cmd.addInt(1);cmd.addInt(1);cmd.addInt(0);  //yellow
    }
    else if (color == "magenta")
    {
        cmd.addInt(0);cmd.addInt(1);cmd.addInt(1);  //magenta
    }
    else
    {
        cmd.addInt(0);cmd.addInt(0);cmd.addInt(1);  //blue
    }

    portToSimWorld.write(cmd);
}

void motorBabbling::cleanWorld()
{
    Bottle cmd;
    cmd.clear();
    cmd.addString("world");
    cmd.addString("del");
    cmd.addString("all");
    portToSimWorld.write(cmd);
}

bool motorBabbling::generateObjRandom(const string &side, Vector &pos)  //pos in sim FoR
{
    if (pos.size()==3)
    {
        // random vector position in world frame
        pos[0] = 0.05 + 0.17*genRandom();   // x= 0.05 -> 0.22 (left)
        pos[1] = 0.65 + 0.25*genRandom();   // y= 0.6 -> 0.85
        pos[2] = 0.13 + 0.25*genRandom();   // z=.13 -> .38 ~ x=-.13 -> -.38 (robot)

        if (pos[0]>0.2)
            pos[2] = min(0.35,pos[2]);

        if (pos[2]>0.35)
            pos[0] = min(0.2,pos[0]);

        if (side=="right")
            pos[0] = -pos[0];   // due to x-axis in sim FoR points to left

        if (pos[2]<.2)
            pos[1] = max(0.7,pos[1]);

        yDebug("[generateObjRandom] pos=%s", pos.toString(3,3).c_str());

        // random color
        unsigned int colorCode = (unsigned int)(6.*genRandom());
        string color = objectColor[colorCode];

        Bottle& output = objPoseDimOutPort.prepare();
        output.clear();
        Vector objDim(3,0.0);

        // random dimension: depending on shape
        double shape = genRandom();

        if (shape<0.5) // sphere
        {
            double radius = 0.02 + 0.04*genRandom();
            createStaticSphere(radius, pos, color);
            for (int i=0; i<3; i++)
                objDim[i] = radius;
        }
        else
        {
            Vector dim(3,0.0);
            for (int8_t i=0; i<3; i++)
                dim[i] = 0.03 + 0.09*genRandom();
            createStaticBox(dim,pos, color);
            for (int i=0; i<3; i++)
                objDim[i] = dim[i];
        }

        // TODO: convert pos to robot frame
        Vector temp(3);
        convertPosFromSimToRootFoR(pos, temp);
        pos = temp;


        addVectorToBottle(pos,output);
        addVectorToBottle(objDim,output);
        objPoseDimOutPort.setEnvelope(ts);
        objPoseDimOutPort.write();

        return true;
    }
    else
        return false;
}

void motorBabbling::convertPosFromSimToRootFoR(const Vector &pos, Vector &outPos)
{
    Vector pos_temp = pos;
    pos_temp.resize(4);
    pos_temp(3) = 1.0;

    outPos.resize(4,0.0);
    outPos = T_root_world * pos_temp;

    outPos.resize(3);
    return;
}

void motorBabbling::addVectorToBottle(const Vector &vec, Bottle &b)
{
    for (int8_t i=0; i<vec.size(); i++)
        b.addDouble(vec[i]);
}
