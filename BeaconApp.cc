#include "veins/modules/application/traci/BeaconApp.h"

using namespace veins;

Define_Module(veins::BeaconApp);

void BeaconApp::initialize(int stage){
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sendBeaconEvt = new cMessage("Send Beacon Event", SEND_BEACON_EVT);
        leaderElectionEvt = new cMessage("Leader Election Event", LEADER_ELECTION_EVT);
        self = getParentModule()->getIndex();
        isLeader = false;
        beaconInterval = par("beaconInterval").doubleValue();
        windowTimeRestorCom = par("windowTime").doubleValue();
        expiration = 0.0;
    }
//    else if (stage == 1) {
//        if(DemoBaseApplLayer::sendBeacons == true){
//            scheduleAt(simTime().dbl() + 0.01, sendBeaconEvt);
//        }
//    }
}

BeaconApp::~BeaconApp() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(leaderElectionEvt);
}

void BeaconApp::finish(){
    DemoBaseApplLayer::finish();
}

void BeaconApp::onBSM(DemoSafetyMessage* bsm){
    switch(bsm->getKind()){
    case SEND_BEACON_EVT:{
        break;
    }
    case LEADER_ELECTION_EVT:{
        expiration = simTime().dbl() + windowTimeRestorCom;
        //std::cout << simTime().dbl()  << " Evento de leader recebido, lider expira em: "
                //<< expiration << endl;
        for(int i = 0; i < bsm->getLeadersArraySize(); i++){
            if(self == bsm->getLeaders(i)){
                isLeader = true;
                break;
            } else{
                isLeader = false;
                cancelEvent(sendBeaconEvt);
                scheduleAt(expiration + 0.1, sendBeaconEvt);
            }
        }
        break;
    }
    default:{
        EV << "onBSM - O tipo da mensagem não foi detectado." << bsm->getKind() << endl;
        break;
    }
    }
}

void BeaconApp::onWSM(BaseFrame1609_4* wsm){
}

void BeaconApp::onWSA(DemoServiceAdvertisment* wsa){
}

void BeaconApp::handleSelfMsg(cMessage* msg){
    switch (msg->getKind()) {
    case SEND_BEACON_EVT:{
        //std::cout << "Send Beacons: " << simTime().dbl() << endl;

        if(isLeader || simTime().dbl() > expiration){
            //std::cout << "Sending: " << mobility->getRoadId().c_str() << endl;
            DemoSafetyMessage* bsm = new DemoSafetyMessage();
            DemoBaseApplLayer::populateWSM(bsm);
            //-- MOBILITY --
            bsm->setSenderPos(mobility->getPositionAt(simTime()));
            bsm->setSenderSpeedDouble(mobility->getSpeed());
            bsm->setRoadID(mobility->getRoadId().c_str());
            //std::cout << "Self: " << self << "direction: " << mobility->getCurrentDirection() << endl;
            //bsm->setCurrentDirection(mobility->getCurrentDirection().x);
            //-- MOBILITY --
            bsm->setSourceAddress(self);
            bsm->setTimestamp(simTime());
            bsm->setKind(SEND_BEACON_EVT);
            DemoBaseApplLayer::sendDelayedDown(bsm, uniform(0.0,1));
            scheduleAt(simTime().dbl() + beaconInterval, sendBeaconEvt);
            break;
        }

    }
    default: {
        EV << "handleSelfMsg - O tipo da mensagem não foi detectado." << endl;
        break;
    }
    }
}

void BeaconApp::handlePositionUpdate(cObject* obj){
    //DemoBaseApplLayer::handlePositionUpdate(obj);
}
