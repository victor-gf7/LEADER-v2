#pragma once

#include "veins/veins.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace omnetpp;

namespace veins {

/**
 * @brief
 * Simple beacon app
 *
 * @author Wellington Viana Lobato Junior
 *
 */

class VEINS_API BeaconApp : public DemoBaseApplLayer {
public:
    ~BeaconApp();
    //MSG TYPE
//    enum selfMessageKinds {
//        SEND_BEACON_EVT,
//        LEADER_ELECTION_EVT,
//    };
    void initialize(int stage) override;
    void finish() override;

protected:
    void onBSM(DemoSafetyMessage* bsm) override;
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    //VAR
    int self;
    bool isLeader;
    bool scheduled;
    double windowTimeRestorCom;
    double expiration;

    //MSG
    cMessage* sendBeaconEvt;
    cMessage* leaderElectionEvt;
};

} // namespace veins
