#ifndef COLLECTDATA_H
#define COLLECTDATA_H

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

#include "collectData_IDL.h"

class collectData : public yarp::os::RFModule, public collectData_IDL
{
protected:
    yarp::os::RpcServer             rpcPort;
    double                          period;
    std::string                     name;
    yarp::os::Stamp                 ts;

    yarp::os::RpcClient     rpcToMotorBabbling;
    yarp::os::RpcClient     rpcToLearning;

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


    //Thrift

    bool babble_then_reach(int32_t _id, const std::string &_part, int32_t _nb_object=1)
    {
        if (_nb_object>=1)
        {
            double delayBetweenReach = 15.0;
            bool ok=false;
            for (int i=0; i<_nb_object; i++)
            {
                ok = sendCmdBabbling();
                yarp::os::Time::delay(7);
                ok = ok & sendCmdLearningPredict();
                yarp::os::Time::delay(3);

                if (_id==-1)    // reach all taxel of part
                {
                    if (_part=="b" or _part=="both")
                    {
                        for (int i=0; i<reprTaxelsHandR.size(); i++)
                        {
                            ok = ok & sendCmdLearningMove(reprTaxelsHandR[i], "h");
                            yarp::os::Time::delay(delayBetweenReach);
                        }
                        for (int i=0; i<reprTaxelsForearm_sim.size(); i++)
                        {
                            ok = ok & sendCmdLearningMove(reprTaxelsForearm_sim[i], "a");
                            yarp::os::Time::delay(delayBetweenReach);
                        }
                    }
                    else if (_part=="h" or _part=="hand")
                    {
                        for (int i=0; i<reprTaxelsHandR.size(); i++)
                        {
                            ok = ok & sendCmdLearningMove(reprTaxelsHandR[i], "h");
                            yarp::os::Time::delay(delayBetweenReach);
                        }
                    }
                    else if (_part=="a" or _part=="arm")
                    {
                        for (int i=0; i<reprTaxelsForearm_sim.size(); i++)
                        {
                            ok = ok & sendCmdLearningMove(reprTaxelsForearm_sim[i], "a");
                            yarp::os::Time::delay(delayBetweenReach);
                        }
                    }
                }
                else
                {
                    ok = ok & sendCmdLearningMove(_id,_part);
                    yarp::os::Time::delay(delayBetweenReach);
                }

                if (!ok)
                    return ok;

            }
            return ok;
        }
        else
            return false;

    }

   
};

#endif // COLLECTDATA_H
