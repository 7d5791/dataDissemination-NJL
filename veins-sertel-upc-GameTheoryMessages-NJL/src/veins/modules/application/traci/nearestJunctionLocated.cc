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

#include "veins/modules/application/traci/nearestJunctionLocated.h"

Define_Module(nearestJunctionLocated)


using std::vector;
using std::string;
using std::map;
using std::list;
using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;


void nearestJunctionLocated::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);

    if (stage == 0) {

        // configuration variables found in omnetpp.ini
        hostToJunctionDistanceThreshold = par("hostToJunctionDistanceThreshold").doubleValue();
        sendWarningInterval = par("sendWarningInterval").doubleValue();
        neighborLifetimeThreshold = par("neighborLifetimeThreshold").doubleValue();
        indexOfAccidentNode = par("indexOfAccidentNode").longValue();
        // end

        traci = TraCIMobilityAccess().get(getParentModule());

        arrivalSignal = registerSignal("arrival");
        arrivalSignal1 = registerSignal("arrival1");

        warningMsgCounterSignalRx = registerSignal("warningMsgCounterRx");
        beaconMsgCounterSignalRx = registerSignal("beaconMsgCounterRx");


        distanceMsgSignalRx = registerSignal("distanceMsgRx");
        numberOfNodesSignal = registerSignal("numberOfNodes");

        receivedData=0;




        sentMessage = false;
        lastDroveAt = simTime();
    }
    else if (stage == 1) {
        list<string> junctionIds = traci->getCommandInterface()->getJunctionIds();

        for (list<string>::iterator i = junctionIds.begin(); i != junctionIds.end(); ++i) {
            string jId = *i;
           // Coord jPos = traci->getManager()->traci2omnet(traci->getCommandInterface()->getJunctionPosition(jId));
            Coord jPos = traci->getCommandInterface()->junction(jId).getPosition();
            junctionMap[jId] = jPos;
        }
    }
}

void nearestJunctionLocated::onBeacon(WaveShortMessage *wsm)
{
    // handle stats

    receivedBeacons++;
    emit(beaconMsgCounterSignalRx,receivedBeacons);

    // is it a new neighbor?
    bool isNewNeighbor = true;
    for (uint i = 0; i < neighbors.size(); ++i) {
        if (neighbors[i]->getTreeId() == wsm->getTreeId())
            isNewNeighbor = false;
    }

    // if it is a new neighbor, store message
    if (isNewNeighbor) {
        neighbors.push_back(wsm->dup());
    }
}

void nearestJunctionLocated::onData(WaveShortMessage *wsm)
{
    // handle stats
    receivedData++;
    emit(warningMsgCounterSignalRx, receivedData);

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // add message to storage
    receivedMessageMap[wsm->getTreeId()].push_back(wsm->dup());

    // is it a new message?
    if (receivedMessageMap[wsm->getTreeId()].size() == 1) {
        std::cerr << "[DEBUG] node: " << getParentModule()->getIndex() << " wsm->getTreeId(): " << wsm->getTreeId() << std::endl;

        findHost()->getDisplayString().updateWith("r=16,green");

        simtime_t delayFirstNewMessage = wsm->getArrivalTime()-wsm->getCreationTime();
     // send a signal
     // statistics recording
        emit(arrivalSignal, delayFirstNewMessage);
    }

    int hops=wsm->getHopCount();
    // statistics recording
    emit(arrivalSignal1, hops);


    distanceFromSource=traci->getPositionAt(simTime()).distance(wsm->getSenderPos());
    // send a signal
    // statistics recording
    emit(distanceMsgSignalRx, distanceFromSource);



    string jId = hostIsInJunction();
    if (jId.empty())
        return;
    if (!hostIsClosestToJunction(jId)) {
        char treeIdStr[64];
        sprintf(treeIdStr, "%ld", wsm->getTreeId());
        cMessage* msg = new cMessage(treeIdStr, SCHEDULED_REBROADCAST_EVT);
        scheduleAt(simTime() + sendWarningInterval, msg);
        return;
    }
}

void nearestJunctionLocated::handleSelfMsg(cMessage *msg)
{
    if (msg->getKind() == SEND_BEACON_EVT) {
        BaseWaveApplLayer::handleSelfMsg(msg);
        return;
    }

    // is it a "rebroadcast" signal?
    else if (msg->getKind() == SCHEDULED_REBROADCAST_EVT) {
        int treeId = atoi(msg->getName());
        WaveShortMessages ms = receivedMessageMap[treeId];

        if ((ms.empty()) || (ms.size() > 1))
            return;
        // yes, disseminate warning message
        sendWSM(ms[0]->dup());
        findHost()->getDisplayString().updateWith("r=18,purple");
    }
}



void nearestJunctionLocated::handlePositionUpdate(cObject *obj)
{
    BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (traci->getSpeed() < 1) {
        if ((simTime() - lastDroveAt >= 10)
                && (!sentMessage)
                && (indexOfAccidentNode == getParentModule()->getIndex())) {

            std::cerr << "[INFO] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

            findHost()->getDisplayString().updateWith("r=16,red");
            sendMessage(traci->getRoadId());
        }
    }
    else {
        lastDroveAt = simTime();
    }
}


void nearestJunctionLocated::sendMessage(std::string blockedRoadId)
{
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
    sentMessage = true;
}

bool nearestJunctionLocated::hostIsClosestToJunction(string junctionId)
{
    // check to see if this host is near an intersection

    Coord jPos = junctionMap[junctionId];

    double hDist = jPos.distance(traci->getPositionAt(simTime()));

    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (jPos.distance(neighbor->getSenderPos()) < hDist) {
            return false;
        }
    }
    return true;
}



string nearestJunctionLocated::hostIsInJunction()
{
    // check to see if this host is near an intersection

    for (map<string,Coord>::iterator i = junctionMap.begin(); i != junctionMap.end(); ++i) {
        string jId = i->first;
        Coord jPos = i->second;
        Coord hPos = traci->getPositionAt(simTime());
        if (jPos.distance(hPos) < hostToJunctionDistanceThreshold) {
            return jId;
        }
    }

    return string();
}


