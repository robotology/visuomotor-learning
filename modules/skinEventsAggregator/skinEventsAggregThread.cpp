#include <algorithm>
#include "skinEventsAggregThread.h"

#define MIN_NUMBER_OF_TAXELS_ACTIVATED 3 //default: 3; to filter out phantoms
#define SKIN_ACTIVATION_MAX_ICUB_SIM 100
#define SKIN_ACTIVATION_MAX_ICUB 30

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

using namespace iCub::skinDynLib;


/*************** public methods section ********************************************/

skinEventsAggregThread::skinEventsAggregThread(int _rate, const string &_name, const string &_robot, int _v) : 
                                               PeriodicThread((double)_rate/1000.0),name(_name), robot(_robot), verbosity(_v) 
{

}

bool skinEventsAggregThread::threadInit()
{
    ts.update();
    skinEventsPortIn.open(("/"+name+"/skin_events:i").c_str());
    skinEvAggregPortOut.open(("/"+name+"/skin_events_aggreg:o").c_str());
   
    if (robot == "icub")
        SKIN_ACTIVATION_MAX = SKIN_ACTIVATION_MAX_ICUB;
    else if (robot == "icubSim")
        SKIN_ACTIVATION_MAX = SKIN_ACTIVATION_MAX_ICUB_SIM;
    else
        yError("skinEventsAggregThread::threadInit(): unknown robot type");
    
    return true;
}

void skinEventsAggregThread::run()
{
    int indexOfBiggestContact = -1;
    skinContact biggestContactInSkinPart;
    Vector geoCenter(3,0.0), normalDir(3,0.0);
    double activation = 0.0;
    Bottle & out = skinEvAggregPortOut.prepare(); out.clear();
    Bottle b, bHand, bForearm, bDetails;
    bDetails.clear();
    b.clear();
    bHand.clear();
    bForearm.clear();
        
    ts.update();
        
    skinContactList *scl = skinEventsPortIn.read(false);
        
    if(scl)
    {
        if(!(scl->empty()))
        {  
            //Probably source of crazy inefficiencies, here just to reach a working state as soon as possible \todo TODO
            map<SkinPart, skinContactList> contactsPerSkinPart = scl->splitPerSkinPart();

            bool collideHand = false, collideForearm = false;
            Vector reprTaxelsForearm, reprTaxelsHand(reprTaxelsHandL.size(),0.0);
            robot=="icubSim"? reprTaxelsForearm.resize(reprTaxelsForearm_sim.size(),0.0) :
                    reprTaxelsForearm.resize(reprTaxelsForearm_real.size(),0.0);

            for(map<SkinPart,skinContactList>::iterator it=contactsPerSkinPart.begin(); it!=contactsPerSkinPart.end(); it++)
            {
                indexOfBiggestContact = getIndexOfBiggestContactInList(it->second);
                
                if (indexOfBiggestContact != -1)
                {
                    bDetails.clear();
                    biggestContactInSkinPart = (it->second)[indexOfBiggestContact];
                    //the output prepared should have identical format to the one prepared in  void vtRFThread::manageSkinEvents()    
                    bDetails.addInt(biggestContactInSkinPart.getSkinPart());
//                    vectorIntoBottle(biggestContactInSkinPart.getGeoCenter(),b);
//                    vectorIntoBottle(biggestContactInSkinPart.getNormalDir(),b);
                    //we add dummy geoCenter and normalDir in Root frame to keep same format as vtRFThread manageSkinEvents 
//                    b.addDouble(0.0); b.addDouble(0.0); b.addDouble(0.0);
//                    b.addDouble(0.0); b.addDouble(0.0); b.addDouble(0.0);
//                    b.addDouble(std::max(1.0,(biggestContactInSkinPart.getPressure()/SKIN_ACTIVATION_MAX))); // % pressure "normalized" with ad hoc constant
                    string skinPartName = biggestContactInSkinPart.getSkinPartName();
                    bDetails.addString(biggestContactInSkinPart.getSkinPartName()); //this one just for readability
                    std::vector<unsigned int> taxelIDs = biggestContactInSkinPart.getTaxelList();

//                    Vector reprTaxelsForearm, reprTaxelsHand(reprTaxelsHandL.size());
//                    robot=="icubSim"? reprTaxelsForearm.resize(reprTaxelsForearm_sim.size()) :
//                            reprTaxelsForearm.resize(reprTaxelsForearm_real.size());

                    for (int i=0; i<taxelIDs.size(); i++)
                    {
                        int j = (int)taxelIDs[i];
                        if( (((skinPartName==SkinPart_s[SKIN_LEFT_FOREARM]) || (skinPartName==SkinPart_s[SKIN_RIGHT_FOREARM])) &&
                            ((j==3) || (j==15)  ||  (j==27) ||  (j==39) ||  (j==51) ||  (j==63) ||  (j==75) ||  (j==87) ||
                            (j==99) || (j==111) || (j==123) || (j==135) || (j==147) || (j==159) || (j==171) || (j==183) ||                          //end of full lower patch: 16 taxels
                            ((j==195) && (robot=="icub")) || (j==207) || ((j==231) && (robot=="icub")) || ((j==255) && (robot=="icubSim")) ||
                            ((j==267) && (robot=="icub")) || (j==291) || (j==303) || ((j==315) && (robot=="icubSim")) || (j==339) || (j==351))   //upper patch: 7 taxels(sim)/8 taxels(real)
                            ))
                        {
                            bDetails.addInt(j);
                            collideForearm = true;
                        }
                        else if((((skinPartName==SkinPart_s[SKIN_LEFT_HAND])) &&
                                ((j==3) || (j==15) || (j==27) || (j==39) || (j==51) //finger: 5 taxels
                                || (j==99) || (j==101) || (j==109) || (j==122) || (j==134))       // left hand: 5 taxels
                                ) ||
                                ((skinPartName==SkinPart_s[SKIN_RIGHT_HAND]) &&
                                ((j==3) || (j==15) || (j==27) || (j==39) || (j==51) //finger
                                || (j==101) || (j==103) || (j==118) || (j==137) || (j==124)))     // right hand
                                )
                        {
                            bDetails.addInt(j);
                            collideHand = true;
                        }


                        if((skinPartName==SkinPart_s[SKIN_LEFT_FOREARM]) || (skinPartName==SkinPart_s[SKIN_RIGHT_FOREARM]))
                        {

                            if (robot=="icubSim")
                            {
                                findIndexOfActivateTaxelandAssign(j,reprTaxelsForearm_sim, reprTaxelsForearm);
                            }
                            else
                            {
                                findIndexOfActivateTaxelandAssign(j, reprTaxelsForearm_real, reprTaxelsForearm);
                            }
                        }
                        else if (skinPartName==SkinPart_s[SKIN_LEFT_HAND])
                        {
                            findIndexOfActivateTaxelandAssign(j, reprTaxelsHandL, reprTaxelsHand);
                        }

                        else if (skinPartName==SkinPart_s[SKIN_RIGHT_HAND])
                        {
                            findIndexOfActivateTaxelandAssign(j, reprTaxelsHandR, reprTaxelsHand);
                        }

                    }
                    yInfo("bDetails = %s",bDetails.toString().c_str());

                    if (collideForearm)
                    {
                        bForearm = bDetails;
                    }
                    else if (collideHand)
                    {
                        bHand = bDetails;
                    }
                }
            }
            b.clear();
            for (int8_t i=0; i<reprTaxelsHand.size(); i++)
                b.addDouble(reprTaxelsHand[i]);
//            out.addList().read(b);
            out.append(b);
            b.clear();
            for (int8_t i=0; i<reprTaxelsForearm.size(); i++)
                b.addDouble(reprTaxelsForearm[i]);
//            out.addList().read(b);
            out.append(b);
            out.addList().read(bHand);
            out.addList().read(bForearm);

            if (collideHand || collideForearm)
            {
                skinEvAggregPortOut.setEnvelope(ts);
                skinEvAggregPortOut.write();
            }

        }
    }
}

void skinEventsAggregThread::findIndexOfActivateTaxelandAssign(const int& taxelID,
                                                               std::vector<int> &reprTaxelsMap,
                                                               Vector &listTaxels)
{
    auto it = std::find(reprTaxelsMap.begin(),reprTaxelsMap.end(),taxelID);
    if (it!=reprTaxelsMap.end())
    {
        int idx = std::distance(reprTaxelsMap.begin(),it);
        listTaxels[idx]=1.0;
    }
}
  

void skinEventsAggregThread::threadRelease()
{
    printMessage(0,"Closing ports..\n");
    skinEventsPortIn.interrupt();
    skinEventsPortIn.close();
    printMessage(1,"skinEventPortIn successfully closed!\n");
    skinEvAggregPortOut.interrupt();
    skinEvAggregPortOut.close();
    printMessage(1,"skinEvAggregPortOut successfully closed!\n");
}

int skinEventsAggregThread::getIndexOfBiggestContactInList(iCub::skinDynLib::skinContactList &sCL)
{
    int index = -1;
    unsigned int maxActivatedTaxels = MIN_NUMBER_OF_TAXELS_ACTIVATED;
    if (! sCL.empty())
    {
        for(skinContactList::iterator c=sCL.begin(); c!=sCL.end(); c++)
        {
            if( c->getActiveTaxels() >= maxActivatedTaxels)
            {
                maxActivatedTaxels = c->getActiveTaxels();
                index = std::distance( sCL.begin(), c);
            }
        }
    }
    //if (index == -1)
      // yError("skinEventsAggregThread::getIndexOfBiggestContactInList: returning index -1");
    
    return index;
}
     
int skinEventsAggregThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);
        return ret;
    }
    else
        return -1;
}
