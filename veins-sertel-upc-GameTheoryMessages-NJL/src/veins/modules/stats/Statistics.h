#ifndef STATISTICS_H
#define STATISTICS_H
#include <csimplemodule.h>

class Statistics : public cSimpleModule
{

public:
    void updateAllBeaconsReceived();
    void updateNewWarningsReceived();
    void updateAllWarningsReceived();
    void updateAllMessagesReceived();

    int getNumberOfAccidentsOccurred() { return numAccidentsOccurred; }
    void incrementAccidentOccurred();

protected:
    uint allBeaconsReceived;
    uint newWarningsReceived;
    uint allWarningsReceived;
    uint allMessagesReceived;
//    bool accidentMode;
//    int numVehicles;

    int numAccidentsOccurred;

    simsignal_t allBeaconsReceivedSignal;
    simsignal_t allNewWarningsReceivedSignal;
    simsignal_t allWarningsReceivedSignal;
    simsignal_t allMessagesReceivedSignal;
    simsignal_t numAccidentsSignal;

protected:
    virtual void initialize(int stage);
//    virtual int numInitStages() const { return 2; }
    virtual void finish();


};

class StatisticsAccess
{
    public:
        StatisticsAccess() {
        }

        Statistics* getIfExists() {
            return dynamic_cast<Statistics*>(simulation.getModuleByPath("stats"));
        }
};

#endif // STATISTICS_H
