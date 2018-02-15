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

#include "veins/modules/application/traci/RCPpp.h"



using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

const simsignalwrap_t RCPpp::parkingStateChangedSignal = simsignalwrap_t(TRACI_SIGNAL_PARKING_CHANGE_NAME);
const double timeSlot = 0.000013;

Define_Module(RCPpp);

void RCPpp::initialize(int stage) {
/*
    // create and open a character archive for output
        std::ofstream ofs("ahmad.txt");
    // create class instance
        const gps_position g(35, 59, 24.567f);
        // save data to archive

             boost::archive::text_oarchive oa(ofs);
             // write class instance to archive
             oa << g;
             // archive and stream closed when destructors are called
**/

    arrivalSignal = registerSignal("arrival");
    arrivalSignal1 = registerSignal("arrival1");

    warningMsgCounterSignalRx = registerSignal("warningMsgCounterRx");
    beaconMsgCounterSignalRx = registerSignal("beaconMsgCounterRx");


    distanceMsgSignalRx = registerSignal("distanceMsgRx");
    numberOfNodesSignal = registerSignal("numberOfNodes");


    // Initialize variables.
    //timeout = 0.5;
    timeoutEvent = new cMessage("timeoutEvent");



	BaseWaveApplLayer::initialize(stage);
	if (stage == 0) {
		mobility = TraCIMobilityAccess().get(getParentModule());
		traci = mobility->getCommandInterface();
		traciVehicle = mobility->getVehicleCommandInterface();
		annotations = AnnotationManagerAccess().getIfExists();
		ASSERT(annotations);



        receivedBeacons = 0;
        receivedData = 0;
        contador=0;

        WATCH(receivedBeacons);
        WATCH(receivedData);
        WATCH(contador);

        listNst.push_back(0);
        listN0t.push_back(0);
        listChannelOccupancy.push_back(0);
        listTimeWindow.push_back(0);

        junctionIds = mobility->getCommandInterface()->getJunctionIds();

        for (list<string>::iterator i = junctionIds.begin(); i != junctionIds.end(); ++i) {
            string jId = *i;
            Coord jPos = mobility->getCommandInterface()->junction(jId).getPosition();
            junctionMap[jId] = jPos;
        }

        //


		sentMessage = false;
		lastDroveAt = simTime();
		findHost()->subscribe(parkingStateChangedSignal, this);
		isParking = false;
		sendWhileParking = par("sendWhileParking").boolValue();

        idMsg=0;
         // configurable variables in omnetpp.ini
         counterThreshold = par("counterThreshold").longValue();
         indexOfAccidentNode = par("indexOfAccidentNode").longValue();
         randomRebroadcastDelay = par("randomRebroadcastDelay").doubleValue();
         randomWaitingTime= par("randomWaitingTime").doubleValue();
         counterWarningMessages=0;
         distanceThreshold = par("distanceThreshold").doubleValue();

         ProbabilityThreshold = par("ProbabilityThreshold").doubleValue();
         //timeout = par("WAIT_TIME").doubleValue();
         //timeout2 = par("delta").doubleValue();

         slots = par("slots").longValue();
         tao = par("tao").doubleValue();

         //
	}
}


RCPpp::RCPpp()
{
    timeoutEvent = nullptr;

}


RCPpp::~RCPpp()
{
    cancelAndDelete(timeoutEvent);

}

double RCPpp::betaCalculation(int k, int n, double alfa)
{
    double betaCalculated =0;
    betaCalculated=pow((k*(pow(alfa,n))),0.5);
    return betaCalculated;
}

void RCPpp::onBeacon(WaveShortMessage* wsm) {

    //Update receivedBeacons and save for statistics
    receivedBeacons++;
    emit(beaconMsgCounterSignalRx,receivedBeacons);


    DBG << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Beacon <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
    DBG << "Received beacon priority  " << wsm->getPriority() << " at " << simTime() << std::endl;
    DBG << "wsm->getSenderAddress()  " << wsm->getSenderAddress() << std::endl;

    //Load IdCar from Beacon Msg
    int b=int(wsm->getCarId());

    //Calculate distance from previous vehicle
    double dsr=(mobility->getPositionAt(simTime()).distance(wsm->getSenderPos()));

/***************************

 */
    double ttl=60;

    EV<<"traciVehicle->getLaneId(): "<<traciVehicle->getLaneId()<<endl;
    //EV<<" traciVehicle->getCurrentRoadOnRoute(): "<< traciVehicle->getCurrentRoadOnRoute()<<endl;
    EV<<"traciVehicle->getLanePosition(): "<<traciVehicle->getLanePosition()<<endl;
    EV<<"traciVehicle->getRoadId(): "<<traciVehicle->getRoadId()<<endl;
    EV<<"traciVehicle->getLaneIndex(): "<<traciVehicle->getLaneIndex()<<endl;
    EV<<"traciVehicle->getTypeId(): "<<traciVehicle->getTypeId()<<endl;


    double lq=getLinkQuality(wsm);
    EV<<"LQ = "<<lq<<endl;

    //distanceFactor
    double d=distanceFactorCalc(dsr,calcDistJoin(wsm),distanceThreshold,vehicleOnJunction());
    EV<<"Distance Factor d = "<<d<<endl;


     /***********************************************/
    //Algorithm to update the weights of the metrics.

    //add time simulation and LQ to  linkQualityFactor map
    linkQualityFactor[simTime()]=lq;

   //calc average of linkQualityFactor map for ttl
    double averageLQ=calAverage(linkQualityFactor,ttl);
    EV<<"averageLQ"<<averageLQ<<endl;

    double r11=0;
    if(lq>0) r11=abs((lq-averageLQ)/lq);
    r1.push_back(r11);
    double s1=calS(r1,r11);
    EV<<"S1 = "<<s1<<endl;



    //add time simulation and d to distanceFactor map
    distanceFactor[simTime()]=d;

    //calc average of distanceFactor map for Thello
    double averageD=calAverage(distanceFactor,ttl);
    EV<<"Average D = "<<averageD<<endl;


    double r22=0;
    if(d>0) r22=abs((d-averageD)/d);
    r2.push_back(r22);

    double s2=calS(r2,r22);

    //Calculate alfa
    double alfa=0;
    if((s1+s2)>0) alfa=1/(s1+s2);


    double wp1=s1*alfa;
    double wp2=s2*alfa;


    EV<<"W_P1 = "<<wp1<<endl;
    EV<<"W_P2 = "<<wp2<<endl;
    uwp1.push_back(wp1);
    uwp2.push_back(wp2);

    EV<<"WP LAST- first= "<<uwp1.back()<<endl;
    EV<<"WP LAST- second= "<<uwp2.back()<<endl;
    p.push_back(lq*wp1+d*wp2);

    EV<<"p = "<<p.back()<<endl;




    wt.push_back((simtime_t)(SLOTLENGTH_11P*timeSlot*(1-p.back())+SimTime(randomWaitingTime,SIMTIME_US)));

    EV<<"waitingTime[WT] on Beacon Message: "<<wt.back()<<endl;

    //Create/Update Neighbors Table
     if (ListBeacon.SearchBeacon(b)){
             ListBeacon.UpdateBeacon(wsm->getCarId(),wsm->getArrivalTime(), wsm->getCreationTime(),
                     wsm->getCarId(),wsm->getSpeed(), wsm->getVecX(),wsm->getVecY(), wsm->getVecZ(),
                     calcDistJoin(wsm),dsr, wt.back(),calcAbe(wsm), wsm->getAngleRad());
     }else{
             ListBeacon.AddBeacon(wsm->getCarId(),wsm->getArrivalTime(),  wsm->getCreationTime(),
                     wsm->getCarId(),wsm->getSpeed(), wsm->getVecX(), wsm->getVecY(), wsm->getVecZ(),
                     calcDistJoin(wsm), dsr,wt.back(),calcAbe(wsm),wsm->getAngleRad());
     }


    DBG << "Beacons time to live TTL"<<std::endl;

    contador=ListBeacon.PurgeBeacons(ttl);

    //order and print Neighbors Table
    ListBeacon.SortBeacons();
    ListBeacon.PrintBeacons();


    EV<<"How many neighbors are present in my coverage area: "<<ListBeacon.CounterBeacons()<<endl;
    EV<<"Angle rad: "<<wsm->getAngleRad()<<endl;
    EV<<"ID vehicle: "<<b<<endl;





}

double RCPpp::forwardingGame(int N,int M, vector<long double> beta){
       double** mat;
       mat = new double*[N];
       for(int i = 0; i < N; ++i)
           mat[i] = new double[M];



// Creating de matrix with the vector B
//https://martin-thoma.com/solving-linear-equations-with-gaussian-elimination/


       for(int i = 0; i < N; ++i)
              for(int j = 0; j < M; ++j)
              {
                 if (i!=j && j!=N) mat[i][j]=beta[i]/(N-1);
                 else if (i!=j && j==N) mat[i][j]=beta[i];
                 else mat[i][j]=1;
              }

       cout <<"MATRIX"<<endl;

       for(int i = 0; i < N; ++i)
              for(int j = 0; j < M; ++j)
              {
                  cout << mat[i][j] << "  ";
                  if(j == M - 1)
                      cout << endl;
              }


       vector<long double> s;
       s=gaussianElimination(mat,N,M);

       cout.precision(17);
       for ( unsigned i=0; i<s.size(); i++){
            cout <<s[i]<<endl;
       }

       double sum_of_elems=0;
       for(vector<long double>::iterator it = s.begin(); it != s.end(); ++it)
           sum_of_elems += *it;

       cout <<"sum_of_elements: "<<sum_of_elems<<endl;

       return sum_of_elems;

}


double RCPpp::getLinkQuality(WaveShortMessage* wsm){
    double cqb=getChannelQuality();
    double cpb=getCollisionProbability();

    EV<<"getChannelQuality = "<<cqb<<endl;
    EV<<"getCollisionProbability = "<<cpb<<endl;

    double sqb=getSignalQuality(wsm);
    EV<<"getSignalQuality = "<<sqb<<endl;

    double lq=sqb*0.5+(1-cpb)*cqb*(1-0.5);

    return lq;


}


double RCPpp::calBeq(double k, double u, double r, double n){

    return pow((k/((u*(1+r*(n-1))))),(1/(n-1)));
}

double RCPpp::calUtility(double ageFactor, double distanceFactor, double linkQualityFactor)
{
    return 0.2*ageFactor+0.6*distanceFactor+0.2*linkQualityFactor;
    //return 10*distanceFactor;
}


double RCPpp::calAgeFactor(double timeElapsed)
{
    return pow(0.999,timeElapsed);
}

//////////////////////////////////////////////////////////////////////

double RCPpp::calS(updatedWeightsMetrics vectorR, double r)
{
   double s=0;
   double max=0;

   max = *max_element(vectorR.begin(), vectorR.end());

   //for (uint i=0; i<vectorR.size();i++){
     //EV<<"Vector"<<"["<<i<<"]" << vectorR[i] << endl;
   //}

    if (max>0) s= r/max;
    return s;
}

double RCPpp::calAverage(vectorMetrics metric, simtime_t tHello)
{

    vector<double> v;
    int count = 0;
    double sum = 0;
    double avg =0;

    if(simTime()>2)
    {
    //extrac info of metric
    for(vectorMetrics::iterator it = metric.begin(); it != metric.end(); ++it) {
        if((it->first)>(it->first-tHello)) v.push_back(it->second);
        }


    //calculate average
    for (unsigned j=0; j<v.size(); j++){
              //cout << v[j] << " ";
              sum += v[j];
              ++count;
          }
    avg = sum / count;
    }



    return avg;

}

void RCPpp::onData(WaveShortMessage* wsm) {
    //Increase and save receivedData
    receivedData++;
    emit(warningMsgCounterSignalRx, receivedData);

    //Load and save number N of nodes
    int nodes= ListBeacon.CounterBeacons();
    emit(numberOfNodesSignal, nodes);

    DBG << ">>>>>>>>>>NEW MESSAGE RECEIVED<<<<<<<<<<"<<std::endl;

    //Load idMsg
    idMsg=wsm->getTreeId();
    EV<<"idMsg= ? "<<idMsg<<endl;


    // add the new message to storage receivedMessages
    receivedMessages[wsm->getTreeId()].push_back(wsm->dup());

    //Create position Pos and load position from received Msg
    Coord Pos;
    Pos.x=wsm->getVecX();
    Pos.y=wsm->getVecY();
    Pos.z=wsm->getVecZ();


    // Calculate distance from source and send a signal for statistics recording
    distanceFromSource=mobility->getPositionAt(simTime()).distance(wsm->getSenderPos());
    emit(distanceMsgSignalRx, distanceFromSource);


    // Calculate distance from previous node
    double distanceFromPreviousNode=Pos.distance(mobility->getPositionAt(simTime()));

    // add the new message to print  table of receivedMessages
    receivedMessagesTable.AddMessage(wsm->getTreeId(),wsm->getArrivalTime(), wsm->getCreationTime(),wsm->getCarId(), wsm->getHopCount(),0,0,0,0,0,distanceFromSource,distanceFromPreviousNode,0,0);

    //Calculate the rate distanceFromSource/distanceThreshold
    double pro;
    (distanceFromSource/distanceThreshold)>1 ? pro=1:pro=distanceFromSource/distanceThreshold;

    //timeout=slots*(1-pro)* tao;
    timeout=wt.back();

    mapForwardingSlottedProbability[wsm->getTreeId()].push_back(timeout);


    EV<<"distanceThreshold= ? "<<distanceThreshold<<endl;
    EV<<"distanceFromSource= ? "<<distanceFromSource<<endl;
    EV<<"distanceFromSource/distanceThreshold= ? "<<pro<<endl;

    EV<<"timeout= ? "<<timeout<<endl;



    //  Delay is calculated only in the first new Message arrived and save a signal for statistics recording
    if (mapForwardingSlottedProbability[wsm->getTreeId()].size()==1){
        simtime_t delayFirstNewMessage = wsm->getArrivalTime()-wsm->getCreationTime();
        emit(arrivalSignal, delayFirstNewMessage);
    }
    // receivedMessagesTable.SortMessages();

    // print receivedMessages table
    receivedMessagesTable.PrintWarningMessages();

    counterWarningMessages=receivedMessagesTable.CounterMessages();

    //Load number of hops from Messages and save for statistics recording
    int hops=wsm->getHopCount();
    emit(arrivalSignal1, hops);

    // prevent originating disseminator from participating in further dissemination attempts
    if (sentMessage)
         return;

    //Display green when vehicle receive a message

    findHost()->getDisplayString().updateWith("r=16,green");
    annotations->scheduleErase(1, annotations->drawLine(wsm->getSenderPos(), mobility->getPositionAt(simTime()), "blue"));

    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getWsmData(), 9999);



    EV<<"TRACI->getRouteIds().pop_back()"<<traci->getRouteIds().back()<<endl;


    EV<<"===============================RCP++================================================"<<endl;

    double d=distanceFactorCalc(mobility->getPositionAt(simTime()).distance(wsm->getSenderPos()),calcDistJoin(wsm),distanceThreshold, vehicleOnJunction());
    double lq=getLinkQuality(wsm);
    EV<<"d en warning data="<<d<<endl;
    EV<<"lq en warning data="<<lq<<endl;

    double ttl=60;

    /***************************************************************************************************/
     /***********************************************/
    //Algorithm to update the weights of the metrics.

    //add time simulation and LQ to  linkQualityFactor map
    linkQualityFactor[simTime()]=lq;

   //calc average of linkQualityFactor map for ttl
    double averageLQ=calAverage(linkQualityFactor,ttl);
    EV<<"averageLQ"<<averageLQ<<endl;

    double r11=0;
    if(lq>0) r11=abs((lq-averageLQ)/lq);
    r1.push_back(r11);
    double s1=calS(r1,r11);
    EV<<"S1 = "<<s1<<endl;



    //add time simulation and d to distanceFactor map
    distanceFactor[simTime()]=d;

    //calc average of distanceFactor map for Thello
    double averageD=calAverage(distanceFactor,ttl);
    EV<<"Average D = "<<averageD<<endl;


    double r22=0;
    if(d>0) r22=abs((d-averageD)/d);
    r2.push_back(r22);

    double s2=calS(r2,r22);

    //Calculate alfa
    double alfa=0;
    if((s1+s2)>0) alfa=1/(s1+s2);


    double wp1=s1*alfa;
    double wp2=s2*alfa;


    EV<<"W_P1 = "<<wp1<<endl;
    EV<<"W_P2 = "<<wp2<<endl;
    uwp1.push_back(wp1);
    uwp2.push_back(wp2);

    EV<<"WP LAST- first= "<<uwp1.back()<<endl;
    EV<<"WP LAST- second= "<<uwp2.back()<<endl;
    p.push_back(lq*wp1+d*wp2);

    EV<<"p = "<<p.back()<<endl;

    wt.push_back((simtime_t)(SLOTLENGTH_11P*timeSlot*(1-p.back())+SimTime(randomWaitingTime,SIMTIME_US)));

    EV<<"waitingTime[WT] on Warning Message : "<<wt.back()<<endl;

    /***************************************************************************************************/


    double difDistance=abs(distanceThreshold-distanceFromPreviousNode);


     simtime_t timeout1=SimTime(randomWaitingTime, SIMTIME_US);  // timeout1

     EV<<"(timeSlot* ="<<timeSlot<<endl;
     EV<<"difDistance)+ ="<<difDistance<<endl;
     EV<<"timeout1 ="<<timeout1<<endl;

     EV<<"individualOffset ="<<individualOffset<<endl;




     timeout=(simtime_t)(wt.back()*difDistance)+timeout1;

     EV<<"(simtime_t)(timeSlot*difDistance)+SimTime(randomWaitingTime, SIMTIME_MS) =timeout="<<timeout<<endl;



    // is it a new warning message?
    ///if (receivedMessagesTable.CounterMessages() == 1 ) {

    if (receivedMessages[wsm->getTreeId()].size() == 1) {
        EV<<"ENVIANDO SELFMSG EN : "<<timeout<<endl;
            // add a random waiting period before proceeding. Please see:
            //     * onSelfMsg for continuation.
            //     * .randomBroadcastDelay configuration in omnetpp.ini


     // scheduleAt sends messege to self (see handleSelfMsg() below and randomRebroadcastDelay in omnetpp.ini

     //scheduleAt(simTime()+timeout,timeoutEvent); SimTime(randomRebroadcastDelay, SIMTIME_MS)
        //SimTime(randomRebroadcastDelay, SIMTIME_MS)+
     scheduleAt(simTime()+timeout ,timeoutEvent);

    }else{
          // add a random waiting period before proceeding. Please see:
          //     * onSelfMsg for continuation.
          //     * .randomBroadcastDelay configuration in omnetpp.ini
          if (timeoutEvent->isScheduled()) {
              // EV<<"CANCELANDO  SELFMSG1"<<endl;
              std::cerr << "[INFO] RE-BROADCAST  CANCELED @simTime: " << simTime().str() << " in node: " << getParentModule()->getIndex() << endl;
              cancelEvent(timeoutEvent);

               }
    }
}

void RCPpp::sendMessage(std::string blockedRoadId) {
    sentMessage = true;

    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(blockedRoadId.c_str());
    // Update Position
    wsm->setVecX(mobility->getPositionAt(simTime()).x);
    wsm->setVecY(mobility->getPositionAt(simTime()).y);
    wsm->setVecZ(mobility->getPositionAt(simTime()).z);
    wsm->setCarId(getParentModule()->getIndex());

    sendWSM(wsm);
}
void RCPpp::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}
void RCPpp::handleParkingUpdate(cObject* obj) {
	isParking = mobility->getParkingState();
	if (sendWhileParking == false) {
		if (isParking == true) {
			(FindModule<BaseConnectionManager*>::findGlobalModule())->unregisterNic(this->getParentModule()->getSubmodule("nic"));
		}
		else {
			Coord pos = mobility->getCurrentPosition();
			(FindModule<BaseConnectionManager*>::findGlobalModule())->registerNic(this->getParentModule()->getSubmodule("nic"), (ChannelAccess*) this->getParentModule()->getSubmodule("nic")->getSubmodule("phy80211p"), &pos);
		}
	}
}
void RCPpp::handlePositionUpdate(cObject* obj) {
	BaseWaveApplLayer::handlePositionUpdate(obj);

    // stopped for for at least 10s?
    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10) {
            findHost()->getDisplayString().updateWith("r=16,red");
            //&& indexOfAccidentNode == getParentModule()->getIndex()
            if (!sentMessage && indexOfAccidentNode == getParentModule()->getIndex()){
                DBG << ">>>>>>>>>>>>>> NEW MESSAGE EMERGENCY GENERATED <<<<<<<<<<<<<"<<std::endl;
                std::cerr << "[INFO] ACCIDENT STARTED @simTime: " << simTime().str() << " for node: " << getParentModule()->getIndex() << endl;
                sendMessage(mobility->getRoadId());

            }
        }
    }
	else {
		lastDroveAt = simTime();
	}
}
void RCPpp::sendWSM(WaveShortMessage* wsm) {
	if (isParking && !sendWhileParking) return;
	sendDelayedDown(wsm,individualOffset);
}

void RCPpp::handleSelfMsg(cMessage *msg)
{

    // for "data" and "beacon" self messages
    if ((!strcmp(msg->getName(), "timeoutEvent"))) {
        if(vectorBeta.size()<4) forwardMessage(1);
        else forwardMessage(1);
        return;

    }else{
        BaseWaveApplLayer::handleSelfMsg(msg);
        return;
    }
}

void RCPpp::forwardMessage(double p)
{
/*
    // if the number of times a warning message is received exceeds the counterThreshold
    // configuration variable, do not rebroadcast.*/
    //if (uniform(0,1) > p)
    //if ((unsigned)counterWarningMessages  >=  (unsigned)counterThreshold)// && uniform(0,1) > p)
   //if (uniform(0,1)>p)
   if(counterWarningMessages>1)
       {
       if (timeoutEvent->isScheduled()) {
           std::cerr << "[INFO] RE-BROADCAST  CANCELED @simTime: " << simTime().str() << " in node: " << getParentModule()->getIndex() << endl;
           cancelEvent(timeoutEvent);
       }
       return;
       }


  EV<<"<< BROADCAST>>"<<endl;

    // Duplicate message and send the copy.
    WaveShortMessage *copy = receivedMessages[idMsg][0]->dup();

    // Increment hop count.
    copy->setHopCount(copy->getHopCount()+1);

    // Update Position

    copy->setVecX(mobility->getPositionAt(simTime()).x);
    copy->setVecY(mobility->getPositionAt(simTime()).y);
    copy->setVecZ(mobility->getPositionAt(simTime()).z);

    copy->setCarId(getParentModule()->getIndex());

    sendWSM(copy);

    // add the new message to storage fowardedMessages
    fowardedMessages[copy->getTreeId()].push_back(copy->dup());

    /* //print fowardedMessages
    EV<<"====FORWARDED MESSAGES===="<<endl;
    printMessages(fowardedMessages);*/



    std::cerr << "[INFO] RE-BROADCAST  STARTED @simTime: " << simTime().str() << " from node: " << getParentModule()->getIndex() << endl;


    return;
}

bool RCPpp::hostIsClosestToJunction(string junctionId)
{
    // check to see if this host is near an intersection

    Coord jPos = junctionMap[junctionId];

    double hDist = jPos.distance(mobility->getPositionAt(simTime()));

    for (uint i = 0; i < neighbors.size(); ++i) {
        WaveShortMessage* neighbor = neighbors[i];
        if (jPos.distance(neighbor->getSenderPos()) < hDist) {
            return false;
        }
    }
    return true;
}

bool RCPpp::vehicleOnJunction()
{
    bool onJunction=false;
    // check to see if this host is near an intersection

    for (map<string,Coord>::iterator i = junctionMap.begin(); i != junctionMap.end(); ++i) {
        string jId = i->first;
        Coord jPos = i->second;
        Coord hPos = mobility->getPositionAt(simTime());
        if (jPos.distance(hPos) < 4) {
            onJunction=true;
            return onJunction;
        }
    }

    return onJunction;
}

string RCPpp::getIdJunction()
{
    // check to see if this host is near an intersection

    for (map<string,Coord>::iterator i = junctionMap.begin(); i != junctionMap.end(); ++i) {
        string jId = i->first;
        Coord jPos = i->second;
        Coord hPos = mobility->getPositionAt(simTime());
        if (jPos.distance(hPos) < 4) {
            return jId;
        }
    }

    return string();
}

double RCPpp::getMin(std::map<std::string, double> mymap)
{
  std::pair<std::string, double> min
      = *min_element(mymap.begin(), mymap.end(), CompareSecond());
  return min.second;
}

double RCPpp::distanceFactorCalc(double d_sr, double d_ri, double r_max, bool  r_vehicleOnIntersection)
{
    // Dsr is the relative distance between source s and receptor r vehicles
    // Dri is the relative distance between vehicle r and the next nearest intersection
    // Rmax is the maximum transmission range
    double d;
    if (!r_vehicleOnIntersection)
        d=d_sr/r_max;
    else
        d=1-(d_ri/(d_ri+1));
    return d;
}

void RCPpp::printPhLayer(WaveShortMessage* wsm) {
        DBG << " >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PHYSIC LAYER<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< "<<std::endl;
        EV<<"getRecvPower_dBm(RSSI)= "<<((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm() <<std::endl;
        EV<<"Bitrate= "<<((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getBitrate() <<std::endl;
        double mySNR=((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getSnr();
        double mySNRLog=10*log10(mySNR);
        EV<<"SNR= "<<mySNR <<std::endl;
        EV<<"SNR[dB]= "<<mySNRLog <<std::endl;
        //EV<<"sensitivity[dBm]= "<<par("sensitivity").doubleValue()<<std::endl;
        //double sensitivity = par("sensitivity").doubleValue();
        //sensitivity = FWMath::dBm2mW(sensitivity);
        PhyLayer80211p* phy11p;
        phy11p = FindModule<PhyLayer80211p*>::findSubModule(
                             getParentModule());
                assert(phy11p);
        double cca=phy11p->getCCAThreshold();
        EV<<"getCCAThreshold= "<<cca<<std::endl;


}

void RCPpp::printMacLayer(WaveShortMessage* wsm) {
    DBG << ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>MAC LAYER <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<"<<std::endl;
    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
           EV<<"myMacp= "<<myMacp<<std::endl;
                  assert(myMacp);
           EV<<"myMacp= "<<myMacp<<std::endl;

           EV<<"Current TxPower= "<<myMacp->getTxPower()<<std::endl;
           EV<<"Current ReceivedPackets= "<<myMacp->getReceivedPackets()<<std::endl;
           EV<<"Current ReceivedBroadcasts= "<<myMacp->getReceivedBroadcasts()<<std::endl;
           EV<<"Current SentPackets= "<<myMacp->getSentPackets()<<std::endl;
           EV<<"Current DroppedPackets= "<<myMacp->getDroppedPackets()<<std::endl;

           EV<<"Current TXRXLostPackets= "<<myMacp->getTXRXLostPackets()<<std::endl;
           EV<<"Current SNIRLostPackets= "<<myMacp->getSNIRLostPackets()<<std::endl;
           EV<<"Current NumTooLittleTime= "<<myMacp->getNumTooLittleTime()<<std::endl;
           EV<<"Current NumInternalContention= "<<myMacp->getNumInternalContention()<<std::endl;
           EV<<"Current NumBackoff= "<<myMacp->getNumBackoff()<<std::endl;
           EV<<"Current SlotsBackoff= "<<myMacp->getSlotsBackoff()<<std::endl;
           EV<<"Current TotalBusyTime= "<<myMacp->getTotalBusyTime()<<std::endl;

           DBG << "******************************************************* "<<std::endl;
           DBG << "--------------------------IdleTimeChannel--------------------------"<<std::endl;
           DBG << "******************************************************* "<<std::endl;
           EV<<"Current MyIdleTime= "<<(simTime().dbl()-myMacp->getTotalBusyTime())/simTime().dbl()<<std::endl;
           EV<<"Current simTIme()= "<<simTime()<<std::endl;
           EV<<"Current getNumSuccessfullTx= "<<myMacp->getNumSuccessfullTx()<<std::endl;
           EV<<"Current gethSlotsBackoff= "<<myMacp->gethSlotsBackoff()<<std::endl;
           EV<<"Current gethNumBackoff= "<<myMacp->gethNumBackoff()<<std::endl;
           DBG << "******************************************************* "<<std::endl;
}

double RCPpp::getChannelQuality() {
    DBG << "::::::::::::  Channel Quality : :::::::::::: "<<std::endl;
    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    assert(myMacp);

    double cq=0;
    double n0t=0;
    double nst=0;
    //number Of Overall Transmisions
    //n0t =double(myMacp->getTXRXLostPackets())+double(myMacp->getSNIRLostPackets())+double(myMacp->getSentPackets());
    n0t =double(myMacp->getTXRXLostPackets())+double(myMacp->getSentPackets());

    //number Of Successfull Transmisions
    nst=double(myMacp->getNumSuccessfullTx());

    EV<<"Current n0t= "<<n0t<<std::endl;
    EV<<"Current nst= "<<nst<<std::endl;
    EV<<"Current push_back(n0t)= "<<listN0t.back()<<std::endl;
    EV<<"Current push_back(nst)= "<<listNst.back()<<std::endl;

    if((n0t-listN0t.back())>0 && (nst-listNst.back())>0)
        cq=(nst-listNst.back())/(n0t-listN0t.back());
    else
        cq=1;

    listNst.push_back(nst);
    listN0t.push_back(n0t);

    EV<<"ChannelQuality := "<<cq<<std::endl;
    DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
    return cq;
}
double RCPpp::getSignalQuality(WaveShortMessage* wsm) {
    DBG << "::::::::::::  Signal Quality : :::::::::::: "<<std::endl;

    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    assert(myMacp);

    // received signal strength

    double rss=pow(10,((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm()/10);
    double rss_db=((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm();
    double rssMax=par("maxTXPower").doubleValue()/1000;
    double rssMax_db=10*log10(rssMax);
    double rssTh=pow(10,par("sensitivity").doubleValue()/10);
    double rssTh_db=par("sensitivity").doubleValue();
    EV<<"Current rss= "<<rss<<std::endl;
    EV<<"Current rss_db= "<<rss_db<<std::endl;

    EV<<"Current rssMax= "<<rssMax<<std::endl;
    EV<<"Current rssMax_db= "<<rssMax_db<<std::endl;

    EV<<"Current rssTh_db= "<<rssTh_db<<std::endl;
    EV<<"Current rssTh= "<<rssTh<<std::endl;

    double s,sq;
    //if(rss>=rssTh)
        s=min((rssTh_db-rss_db)/(rssTh_db-rssMax_db),1.0);
   // else
      //  s=0;

    double snr=10*log10(((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getSnr());

    //speed
    TraCIMobility* mySpeed = FindModule<TraCIMobility*>::findSubModule(getParentModule());
    assert(mySpeed);
    double v=mySpeed->getSpeed()/par("maxSpeedUrban").doubleValue();

    if(snr>0)
        sq=max(0.0,s*(1-1/snr)*(1-v));
    else
        sq=s;

    EV<<"Current v= "<<v<<std::endl;
    EV<<"Current snr= "<<snr<<std::endl;
    EV<<"Current s= "<<s<<std::endl;
    EV<<"Current sq= "<<sq<<std::endl;

    EV<<"Signal Quality := "<<sq<<std::endl;
    DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
   return sq;
}
double RCPpp::getCollisionProbability() {

    DBG << "::::::::::::  Collision Probability : :::::::::::: "<<std::endl;
    Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
    assert(myMacp);

    EV<<"Current TotalBusyTime= "<<myMacp->getTotalBusyTime()<<std::endl;
    EV<<"Current simTime()= "<<simTime()<<std::endl;
    simtime_t cp=0;

    //Channel Occupancy time
    simtime_t co =myMacp->getTotalBusyTime();

    //last time window
    simtime_t tw=simTime();

    EV<<"Channel Occupancy time= "<<co<<std::endl;
    EV<<"last time window= "<<tw<<std::endl;

    EV<<"Channel Occupancy time (anterior)= "<<listChannelOccupancy.back()<<std::endl;
    EV<<"last time window (anterior)= "<<listTimeWindow.back()<<std::endl;
    cp=0;
    (tw.dbl()==listTimeWindow.back().dbl()) ? cp=0: cp=abs((co.dbl()-listChannelOccupancy.back().dbl())/(tw.dbl()-listTimeWindow.back().dbl()));


    listChannelOccupancy.push_back(co);
    listTimeWindow.push_back(tw);
    EV<<"Collision Probability := "<<cp<<std::endl;
    DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
    return cp.dbl();
}

void RCPpp::printTracyLayer(WaveShortMessage* wsm) {

    DBG << ">>>>>>>>>>>>>VELOCIDAD<<<<<<<<<<<<<<<< "<<std::endl;
    TraCIMobility* mySpeed = FindModule<TraCIMobility*>::findSubModule(getParentModule());
    assert(mySpeed);
    EV<<"Current Speed= "<<mySpeed->getSpeed()<<std::endl;
    EV<<"Current maxSpeedUrban= "<<par("maxSpeedUrban").doubleValue()<<std::endl;

    EV<<"Current maxTXPower= "<<par("maxTXPower").doubleValue()<<std::endl;
    EV<<"Current sensitivity= "<<par("sensitivity").doubleValue()<<std::endl;

    EV<<"Current getAngleRad= "<<mySpeed->getAngleRad()<<std::endl;

    EV<<"Current getCurrentPosition().x= "<<mySpeed->getCurrentPosition().x<<std::endl;
    EV<<"Current getCurrentPosition().y= "<<mySpeed->getCurrentPosition().y<<std::endl;
    EV<<"Current getCurrentPosition().z= "<<mySpeed->getCurrentPosition().z<<std::endl;
    EV<<"Current getIndex = "<<getParentModule()->getIndex()<<std::endl;

    double maxSpeed=10;
    EV<<"Current MaxSpeed= "<<maxSpeed<<std::endl;

}
double RCPpp::calcAbe(WaveShortMessage* wsm)
{

        TraCIMobility* mySpeed = FindModule<TraCIMobility*>::findSubModule(getParentModule());
        Mac1609_4* myMacp = FindModule<Mac1609_4*>::findSubModule(getParentModule());
        string myroad=mobility->getRoadId();
        DBG << "Route :" << myroad<<std::endl;
        list<string> junctionIds = mobility->getCommandInterface()->getJunctionIds();


        DBG << "CWMIN_11P :" <<CWMIN_11P<<std::endl;
        DBG << "CWMAX_11P :" << CWMAX_11P<<std::endl;


        double k=(((2 * (SLOTLENGTH_11P + SIFS_11P).dbl())+(double(myMacp->gethNumBackoff())*SLOTLENGTH_11P.dbl()))/par("beaconInterval").doubleValue());

        DBG << "k :" << k<<std::endl;
        // How to calc ABE

        double f_m_N_s=-7.47*10e-5 *par("headerLength").doubleValue()-8.98*10e-3*double(ListBeacon.CounterBeacons())-1.42*10e-3*double(mySpeed->getSpeed())+1.98;
        f_m_N_s>1 ? f_m_N_s=0:f_m_N_s;

        double ts=wsm->getTimeIdleChannel().dbl();
        double tr=(simTime().dbl()-myMacp->getTotalBusyTime().dbl())/simTime().dbl();
        double myBitrate=((DeciderResult80211*)((PhyToMacControlInfo*)wsm->getControlInfo())->getDeciderResult())->getBitrate();
        DBG << "ts :" << ts<<std::endl;
        DBG << "tr :" << tr<<std::endl;
        DBG << "Bitrate :" << myBitrate<<std::endl;
        DBG << "f_m_N_s :" << f_m_N_s<<std::endl;

        double cpb=getCollisionProbability();
        DBG << "cpb :" << cpb<<std::endl;

        double  abe1=double((1-k)*(1-f_m_N_s)*ts*tr*myBitrate);
        double  abe2=double((1-k)*(1-cpb)*ts*tr*myBitrate);
        double  availability= abe2/myBitrate;
       DBG << "::::::::::::::::::::::::ABE1::::::::::::::::::::::::" << abe1<<std::endl;
       DBG << "::::::::::::::::::::::::ABE2::::::::::::::::::::::::" << abe2<<std::endl;
       DBG << "::::::::::::::::::::::::availability::::::::::::::::::::::::" << availability<<std::endl;
       DBG << ":::::::::::: ::::::::::: :::::::::::: "<<std::endl;
       return availability;
}
double RCPpp::calcDistJoin(WaveShortMessage* wsm)
{
    string jId = getIdJunction();
     DBG  << "JUNCTIONS" << std::endl;

     DBG  << jId << std::endl;


     for (list<string>::iterator i = junctionIds.begin(); i != junctionIds.end(); ++i) {
                    string jId = *i;
                    Coord jPos = mobility->getCommandInterface()->junction(jId).getPosition();
                    Coord hPos = mobility->getPositionAt(simTime());
                    junctionDistance[jId] = jPos.distance(hPos);
                }
     double distJ=getMin(junctionDistance);

     if (distJ>5){
             DBG  << "NO Junctions" << std::endl;
         }
     if (distJ<=5) {
             DBG  << "Junctions :" <<distJ<<" m."<<std::endl;
             DBG  << "*********************************" << std::endl;
         }

     return distJ;

}

/*******************************************************************************************/
// function to reduce matrix to r.e.f.  Returns a value to
// indicate whether matrix is singular or not
int RCPpp::forwardElim(double** mat, int N, int M){

    for (int k=0; k<N; k++)
    {
        // Initialize maximum value and index for pivot
        int i_max = k;
        int v_max = mat[i_max][k];

        /* find greater amplitude for pivot if any */


     for (int i = k+1; i < N; i++)
            if (abs(mat[i][k]) > v_max)
                v_max = mat[i][k], i_max = i;

        /* if a prinicipal diagonal element  is zero,
         * it denotes that matrix is singular, and
         * will lead to a division-by-zero later. */


    if (!mat[k][i_max])
            return k; // Matrix is singular

        /* Swap the greatest value row with current row */


    if (i_max != k)
            swap_row(mat,N,M, k, i_max);


        for (int i=k+1; i<N; i++)
        {
            /* factor f to set current row kth elemnt to 0,
             * and subsequently remaining kth column to 0 */
            double f = mat[i][k]/mat[k][k];

            /* subtract fth multiple of corresponding kth
               row element*/
            for (int j=k+1; j<=N; j++)
                mat[i][j] -= mat[k][j]*f;

            /* filling lower triangular matrix with zeros*/
            mat[i][k] = 0;
        }

        //print(mat);        //for matrix state
    }
    //print(mat);            //for matrix state
    return -1;
}


vector<long double> RCPpp::backSub(double** mat, int N, int M){
    // function to calculate the values of the unknowns
    vector<long double> v;
    double x[N];  // An array to store solution

    /* Start calculating from last equation up to the
       first */


    for (int i = N-1; i >= 0; i--)
    {
        /* start with the RHS of the equation */


        x[i] = mat[i][N];

        /* Initialize j to i+1 since matrix is upper
           triangular*/


        for (int j=i+1; j<N; j++)
        {
            /* subtract all the lhs values
             * except the coefficient of the variable
             * whose value is being calculated */



            x[i] -= mat[i][j]*x[j];
        }

        /* divide the RHS by the coefficient of the
           unknown being calculated */


        x[i] = x[i]/mat[i][i];
    }

    printf("\nSolution for the system:\n");
    for (int i=0; i<N; i++){
        v.push_back(x[i]);
        printf("%lf\n", x[i]);
    }

    return v;

}


vector<long double>  RCPpp::gaussianElimination(double** mat, int N, int M){
    // function to get matrix content

    /* reduction into r.e.f. */


    int singular_flag = forwardElim(mat,N,M);

    /* if matrix is singular */


    if (singular_flag != -1)
    {
        printf("Singular Matrix.\n");

        /* if the RHS of equation corresponding to
           zero row  is 0, * system has infinitely
           many solutions, else inconsistent*/

        if (mat[singular_flag][N])
            printf("Inconsistent System.");
        else
            printf("May have infinitely many "
                   "solutions.");
        vector<long double> vE;

        return vE;
    }

    /* get solution to system and print it using
       backward substitution */
    vector<long double> v1 = backSub(mat,N,M);
    return v1;
}



void  RCPpp::swap_row(double** mat, int N, int M, int i, int j){
    // function for elemntary operation of swapping two rows
    //printf("Swapped rows %d and %d\n", i, j);

     for (int k=0; k<=N; k++)
     {
         double temp = mat[i][k];
         mat[i][k] = mat[j][k];
         mat[j][k] = temp;
     }

}


void RCPpp::print(double** mat, int N, int M){
    // function to print matrix content at any stage
    for (int i=0; i<N; i++, printf("\n"))
        for (int j=0; j<=N; j++)
            printf("%lf ", mat[i][j]);

    printf("\n");

}


