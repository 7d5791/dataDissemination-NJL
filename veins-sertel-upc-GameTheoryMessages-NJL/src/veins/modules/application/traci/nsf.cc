#include "nsf.h"
#include <WaveShortMessage_m.h>
#include <iostream>

using Veins::TraCIMobility;
using Veins::TraCIMobilityAccess;
using std::map;
using std::vector;
using std::string;


Define_Module(nsf)

void nsf::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {

        // configuration variables found in omnetpp.ini
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

        receivedData=receivedBeacons=0;

        lastDroveAt = simTime();
        sentMessage = false;
    }
}

void nsf::receiveSignal(cComponent *source, simsignal_t signalID, cComponent::cObject *obj)
{
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}


void nsf::onBeacon(WaveShortMessage *wsm)
{
    // handle stats
    receivedBeacons++;
    emit(beaconMsgCounterSignalRx,receivedBeacons);

    // is it a new neighbor?
    bool isNewNeighbor = true;
    vector<uint> indices;
    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];

        if (neighbor->getTreeId() == wsm->getTreeId()) {
            isNewNeighbor = false;
            neighbors[i] = wsm;
        }
        else {
            // check for removal
            if (simTime() - neighbor->getArrivalTime() > neighborLifetimeThreshold)
                indices.push_back(i);
        }
    }

    // if it is a new neighbor
    if (isNewNeighbor) {
        for (map<long,WaveShortMessages>::iterator i = receivedWarningMessageMap.begin(); i != receivedWarningMessageMap.end(); ++i) {
            WaveShortMessage* msg = i->second[0];
            ASSERT(msg);
            // disseminate warning message
            sendWSM(msg->dup());
        }
        // add new neighbor to neighbors list
        neighbors.push_back(wsm->dup());
    }

    // remove the old neighbors
    WaveShortMessages newNeighborList;
    for (uint i = 0; i < neighbors.size(); ++i) {
        bool keepNeighbor = true;
        for (uint j = 0; j < indices.size(); ++j) {
            if (i == indices[j])
                keepNeighbor = false;
        }
        if (keepNeighbor)
            newNeighborList.push_back(neighbors[i]);
    }
    neighbors = newNeighborList;
}


void nsf::onData(WaveShortMessage *wsm)
{
    // handle stats
    receivedData++;
    emit(warningMsgCounterSignalRx, receivedData);

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
        return;

    // check if new warning
    bool isNewWarning = true;
    for (map<long,WaveShortMessages>::iterator i = receivedWarningMessageMap.begin(); i != receivedWarningMessageMap.end(); ++i) {
        WaveShortMessage* msg = i->second[0];
        if (msg->getTreeId() == wsm->getTreeId())
            isNewWarning = false;
    }

    if (isNewWarning) {

        findHost()->getDisplayString().updateWith("r=16,green");
        // handle stats
        simtime_t delayFirstNewMessage = wsm->getArrivalTime()-wsm->getCreationTime();
        // send a signal
        // statistics recording
        emit(arrivalSignal, delayFirstNewMessage);
    }

    int hops=wsm->getHopCount();
    // statistics recording
    emit(arrivalSignal1, hops);


    // if there are more than 1 neighbor, disseminate warning
    if (neighbors.size() > 1) {
        sendWSM(wsm->dup());
    }
    // add warning message to received messages storage
    receivedWarningMessageMap[wsm->getTreeId()].push_back(wsm->dup());
}

void nsf::handlePositionUpdate(cComponent::cObject *obj)
{
    // stopped for for at least 10s?
    if (traci->getSpeed() < 1) {
        if ((simTime() - lastDroveAt >= 10)
                && (!sentMessage)
                && (indexOfAccidentNode == getParentModule()->getIndex())) {

            std::cerr << "[DEBUG] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;

            findHost()->getDisplayString().updateWith("r=16,red");
            if (!sentMessage)
                sendMessage(traci->getRoadId());
        }
    }
    else {
        lastDroveAt = simTime();
    }
}

void nsf::sendMessage(std::string blockedRoadId)
{
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    sendWSM(wsm);
}

