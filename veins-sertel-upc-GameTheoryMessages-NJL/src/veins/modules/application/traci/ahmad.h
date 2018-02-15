//
// Copyright (C) 2013-2017 Cristhian Iza <7d5791@gmail.com>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef AHMAD_H_
#define AHMAD_H_

#include <fstream>

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include <cstdlib>
#include <iostream>
#include <vector>




using namespace std;
//
class metrics
{
private:
    double linkQualityFactor;
    double distanceFactor;
    double ageFactor;
    double bandwidth;
    double signalQuality;
    double collisionProbability;
    double channelQuality;
    int numberOfNodes;
public:
    metrics(){};
    metrics(double lqf, double df, double af, double abe, double sq, double cp, double chq, double non) :
        linkQualityFactor(lqf), distanceFactor(df), ageFactor(af), bandwidth(abe), signalQuality(sq),
        collisionProbability(cp), channelQuality(chq), numberOfNodes(non)
    {}
    double getLinkQualityFactor(){return linkQualityFactor;}
    void setLinkQualityFactor(double lqf){linkQualityFactor=lqf;}

    double getdistanceFactor(){return distanceFactor;}
    void setdistanceFactor(double df){distanceFactor=df;}

    double getAgeFactor(){return ageFactor;}
    void setAgeFactor(double af){ageFactor=af;}

    double getBandwidth(){return bandwidth;}
    void setBandwidth(double abe){bandwidth=abe;}

    double getSignalQuality(){return signalQuality;}
    void setSignalQuality(double sq){signalQuality=sq;}

    double getCollisionProbability(){return collisionProbability;}
    void setCollisionProbability(double cp){collisionProbability=cp;}

    double getChannelQuality(){return channelQuality;}
    void setChannelQuality(double chq){channelQuality=chq;}

    double getNumberOfNodes(){return numberOfNodes;}
    void setNumberOfNodes(double non){numberOfNodes=non;}
};


#endif /* AHMAD_H_ */
