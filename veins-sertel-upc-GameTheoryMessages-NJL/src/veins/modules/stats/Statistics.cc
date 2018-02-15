#include "Statistics.h"
#include <iostream>
using std::cerr;
using std::endl;

Define_Module(Statistics)



void Statistics::initialize(int stage)
{
    if (stage == 0) {
        allBeaconsReceivedSignal = registerSignal("allBeaconsReceivedSignal");
        allNewWarningsReceivedSignal = registerSignal("allNewWarningsReceivedSignal");
        allWarningsReceivedSignal = registerSignal("allWarningsReceivedSignal");
        allMessagesReceivedSignal = registerSignal("allMessagesReceivedSignal");
//        accidentStartTimeSignal = registerSignal("accidentStartTimeSignal");
//        percentOfInformedVehiclesSignal = registerSignal("percentOfInformedVehiclesSignal");

        allBeaconsReceived = allWarningsReceived = newWarningsReceived = allMessagesReceived = 0;
//        accidentMode = false;

//        numVehicles = 20;
        numAccidentsOccurred = 0;
    }
}


void Statistics::finish()
{
}


void Statistics::updateAllBeaconsReceived()
{
    ++allBeaconsReceived;
    emit(allBeaconsReceivedSignal, allBeaconsReceived);
}

void Statistics::updateNewWarningsReceived()
{
    ++newWarningsReceived;
    emit(allNewWarningsReceivedSignal, newWarningsReceived);
//    cerr << "num warnings: " << newWarningsReceived << simTime().str() << endl;
}

void Statistics::updateAllWarningsReceived()
{
    emit(allWarningsReceivedSignal, ++allWarningsReceived);
}

void Statistics::updateAllMessagesReceived()
{
    emit(allMessagesReceivedSignal, ++allMessagesReceived);
}

void Statistics::incrementAccidentOccurred()
{
    emit(numAccidentsSignal, ++numAccidentsOccurred);
}
