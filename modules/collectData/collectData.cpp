#include "collectData.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

bool collectData::configure(yarp::os::ResourceFinder &rf) {
    bool bEveryThingisGood = true;

    name = rf.check("name", Value("collectData"), "module name (string)").asString();
    period=rf.check("period",Value(0.0)).asDouble();    // as default, update module as soon as receiving new parts from skeleton2D

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


    setName(name.c_str());

    // Open handler port
    if (!rpcPort.open("/" + getName() + "/rpc"))
    {
        yError() << getName() << ": Unable to open port " << "/" << getName() << "/rpc";
        bEveryThingisGood = false;
    }

    attach(rpcPort);
    
    return bEveryThingisGood;
}

bool    collectData::interruptModule()
{
    rpcPort.interrupt();
    return true;
}

bool    collectData::close()
{
    yInfo() << "Closing module, please wait ... ";

    rpcPort.close();
    yInfo() << "Bye!";

    return true;
}

bool    collectData::updateModule()
{
    ts.update();
    

    return true;
}

double  collectData::getPeriod()
{
    return period;
}

bool    collectData::attach(RpcServer &source)
{
    return this->yarp().attachAsServer(source);
}

bool    collectData::sendCmdBabbling()
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

bool    collectData::sendCmdLearningPredict()
{
    Bottle cmd, rep;
    cmd.addString("predict");

    if (rpcToLearning.write(cmd, rep))
        return true;
    else
        return false;
}

bool    collectData::sendCmdLearningMove(const int &id, const string &partName)
{
    yInfo("send rpc cmd move to taxel %d in %s",id, partName.c_str());
    Bottle cmd, rep;
    cmd.addString("move");
    cmd.addInt(id);
    cmd.addString(partName);

    if (rpcToLearning.write(cmd, rep))
    {
        yInfo("%s",rep.get(0).asString().c_str());
        return true;
    }
    else
        return false;
}

