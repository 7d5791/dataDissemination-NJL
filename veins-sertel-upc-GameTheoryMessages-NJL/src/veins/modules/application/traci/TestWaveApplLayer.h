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

#ifndef TESTWAVEAPPLLAYER_H_
#define TESTWAVEAPPLLAYER_H_
#include "veins/modules/application/traci/beacon.h"
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include <cstdlib>
#include <iostream>
#include <vector>


using namespace std;

using Veins::TraCIMobility;
using Veins::TraCICommandInterface;
using Veins::AnnotationManager;


#ifndef DBG
#define DBG EV
#endif


#define SCHEDULED_REBROADCAST
typedef std::vector<WaveShortMessage*> WaveShortMessages;
typedef std::pair<std::string, double> MyPairType;
//#define DBG std::cerr << "[" << simTime().raw() << "] " << getParentModule()->getFullPath()

/**
 * @brief
 * Minimal WAVE application example.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 */
class TestWaveApplLayer  :  public BaseWaveApplLayer {
    public:
    struct CompareSecond
    {
        bool operator()(const MyPairType& left, const MyPairType& right) const
        {
            return left.second < right.second;
        }
    };



        virtual ~TestWaveApplLayer();

        virtual void initialize(int stage);
        virtual void finish();

    protected:
        virtual void onBeacon(WaveShortMessage* wsm);
        virtual void onData(WaveShortMessage* wsm);
        virtual void handlePositionUpdate(cObject *obj);
        virtual void sendMessage(std::string blockedRoadId);
        virtual void handleSelfMsg(cMessage *msg);

        bool hostIsClosestToJunction(std::string junctionId);
        double  getMin(std::map<std::string, double> mymap);
        double calcAbe(WaveShortMessage* wsm);
        double calcDistJoin(WaveShortMessage* wsm);
        double getChannelQuality();
        double getCollisionProbability();
        double getSignalQuality(WaveShortMessage* wsm);
        void printPhLayer(WaveShortMessage* wsm);
        void printTracyLayer(WaveShortMessage* wsm);
        void printMacLayer(WaveShortMessage* wsm);
        std::string hostIsInJunction();

    protected:
        TraCIMobility* mobility;
        uint32_t receivedBeacons;
        uint32_t receivedData;
        BeaconList ListBeacon;
        int contador;

        std::list<int> listNst;
        std::list<int> listN0t;

        std::list<simtime_t> listChannelOccupancy;
        std::list<simtime_t> listTimeWindow;

        std::list<std::string> junctionIds;
        std::map<std::string,Coord> junctionMap;
        std::map<std::string,double> junctionDistance;
        std::map<long,WaveShortMessages> receivedMessageMap;
        std::vector<WaveShortMessage*> neighbors;
        vector<WaveShortMessage*> warningMessages;

        bool sentMessage;
        long counterThreshold;
        long indexOfAccidentNode;
        double randomRebroadcastDelay;
        map<long,WaveShortMessages> receivedMessages;    // treeId, WSM vector
        simtime_t lastDroveAt;




};

#endif /* TESTWAVEAPPLLAYER_H_ */
