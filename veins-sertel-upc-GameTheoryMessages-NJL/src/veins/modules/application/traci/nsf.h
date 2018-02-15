//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
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

#ifndef nsf_H
#define nsf_H


#include "BaseWaveApplLayer.h"
#include "modules/mobility/traci/TraCIMobility.h"

#include <vector>
#include <map>

using Veins::TraCIMobility;
using Veins::AnnotationManager;
using std::vector;
using std::map;

typedef std::vector<WaveShortMessage*> WaveShortMessages;

class nsf : public BaseWaveApplLayer
{

public:
    virtual void initialize(int stage);
    virtual void receiveSignal(cComponent *source, simsignal_t signalID, cObject *obj);

protected:
    TraCIMobility* traci;

    vector<WaveShortMessage*> warningMessages;

    uint32_t receivedBeacons;
    uint32_t receivedData;
    uint32_t numberOfNodes;

    double distanceFromSource;

    simsignal_t arrivalSignal;
    simsignal_t arrivalSignal1;
    simsignal_t warningMsgCounterSignalRx;
    simsignal_t beaconMsgCounterSignalRx;
      //simsignal_t warningMsgCounterSignalTx;
      //simsignal_t beaconMsgCounterSignalTx;
    simsignal_t distanceMsgSignalRx;
    simsignal_t numberOfNodesSignal;



    simtime_t lastDroveAt;
    bool sentMessage;
    double neighborLifetimeThreshold;
    long indexOfAccidentNode;

    WaveShortMessages                   neighbors;
    std::map<long,WaveShortMessages>    receivedWarningMessageMap;    // treeId, WSM vector

protected:
    virtual void onBeacon(WaveShortMessage *wsm);
    virtual void onData(WaveShortMessage *wsm);
    virtual void handlePositionUpdate(cObject *obj);
    virtual void sendMessage(std::string blockedRoadId);
//    virtual void handleSelfMsg(cMessage *msg);
};

#endif // nsf_H
