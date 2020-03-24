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
#include <yarp/os/all.h>
#include "reachObjMultiDemo.h"

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        yError("Yarp network doesn't seem to be available!");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("kinStructureLearning");
    rf.setDefaultConfigFile("reachObjMultiDemo.ini");
    rf.configure(argc,argv);

    reachObjMultiDemo mod;
    return mod.runModule(rf);
}

