package org.car2x.veins.modules.stats;


simple Statistics
{
    parameters:
        @class(Statistics);
        @display("i=block/table2_vl.png");

        @signal[allBeaconsReceivedSignal](type=long);
        @statistic[allBeaconsReceivedSignal](record=count,vector; description="All Beacons Received");

        @signal[allNewWarningsReceivedSignal](type=long);
        @statistic[allNewWarningsReceivedSignal](record=count,vector; description="All New Warnings Received");

        @signal[allWarningsReceivedSignal](type=long);
        @statistic[allWarningsReceivedSignal](record=count,vector; description="All Warnings Received");

        @signal[allMessagesReceivedSignal](type=long);
        @statistic[allMessagesReceivedSignal](record=count,vector; description="All Messages Received");

        //@signal[numAccidentsSignal](type=long);
        //@statistic[numAccidentsSignal](record=count,vector; description="Number Of Occurred Accidents");
}
