#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "dbscan.h"
#include <numeric>
#include <vector>
#include <queue>
#include <stack>
#include <map>
#include <set>
#include <list>

#include <cstring>
#include <sstream>
#include <algorithm>
#include <limits>
#define MINIMUM_POINTS 4     // minimum number of cluster
#define EPSILON_DBSCAN (20.0)  //distance for clustering, metre^2
#define LEADERS_QTD 1      //number of leaders to be elected

namespace veins {

typedef struct Leader_
{
    vector<int> selfID; // Vehicle ID
    vector<double> value; // representativeness value
}Leader;

struct comp {
    template <typename T>

    // Comparator function
    bool operator()(const T& l,
                    const T& r) const
    {
        if (l.second != r.second) {
            return l.second > r.second;
        }
        return l.first > r.first;
    }
};

class VEINS_API LeaderApp : public DemoBaseApplLayer {
public:
    ~LeaderApp();

protected:
    //MSG TYPE
//    enum selfMessageKinds {
//        LEADER_ELECTION_EVT,
//        MAKE_DBSCAN_EVT,
//        BEACON_MSG_EVT,
//    };
    // The new adjacency list type.
    typedef std::vector<std::vector<int> > adjacency_list;

    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onBSM(DemoSafetyMessage* bsm) override;
    void handleSelfMsg(cMessage* msg) override;
    void resetVariables(int src, int n, list<int> *pred, vector<int> &sigma, vector<float> &delta);
    float bfs_SSSP(int src, int n, stack<int> &visitStack, vector<int> &sigma, list<int> *pred, const adjacency_list &adjList);
    void conductElection(std::map<int, float> map);
    std::map<int, float> sort(map<int, float> map);

    //VAR
    int eventoEscalonado = 0;
    int self;

    double windowTime;
    double actualTime;
    double sumSpeed;
    double meanSpeed;
    double sumLOS;
    double meanLOS;
    double distAB;
    int sizeMap;
    int iDBSCAN; //Apenas um iterador para o DBSCAN
    int contador;
    int randNumber;
    int messageCounter = 0;
    int requesterNumber, cRN;
    int executerNumber, cEN;
    std::map<int,double> mapIDSpeed;
    std::map<int,std::string> mapDBSCAN;
    std::map<std::string,int> DBSCANmap;
    std::map<int,std::list<int>> Clustermap;
    std::map<int, int> mapNodeByCluster;
    std::map<int, Leader> leadersMap;
    std::map<int, Leader>::iterator itLeadersMap;
    //std::map<int,Point> mapVehicles;
    std::map<int,std::string>::iterator itmapDBSCAN;
    std::map<std::string,int>::iterator itDBSCANmap;
    std::map<int,std::list<int>>::iterator itClustermap;
    std::list<int>::iterator itList;
    std::list<int>::iterator jtList;
    string posString;
    string leaderString;
    string fileName;
    Point ponto;
    Leader leader;
    vector<Point> points;
    vector<Point> pointsResult;
    std::vector<std::string> positionByNode;
    std::vector<std::string> positionVizinhoA;
    std::vector<std::string> positionVizinhoB;
    std::string strategy;
    std::list<std::string> listLaneIDs;
    std::list<std::string>::iterator itListLaneIDs;

    std::vector<int>::iterator itClusterIDVector;

    //FROM OTHER CODE
    std::map<std::string, float> edgeBetweenness;
    std::vector<float> nodeBetweenness;
    std::vector<float> closeness;
    std::vector<float> random;

    std::map<int, float> nodeBetweennessMap;
    std::map<int, float> closenessMap;
    std::map<int, float> randomMap;
    std::map<int, float>::iterator itStrategyMap;
    std::vector<int> sigma;
    std::vector<float> delta;
    std::stack<int> visitStack; // Stack that holds the inverse order of visited nodes.
    int e = 0; // Total number of edges (for statistics).
    int start, end, weight;

    bool isWeigthed; // Weighted graph check.
    adjacency_list adjList; // Adjacency list.



    //WRITE FILE
    ofstream outputFile;
    ofstream rawDataFile;
    ofstream leadersFile;
    ofstream plotClusters;
    ofstream counterFile;
    ofstream testeFile;

    //TRACI
    TraCIScenarioManager* manager;
    TraCICommandInterface* traci;

    //MSG
    cMessage* sendCloudMSGEvt;
    cMessage* leaderElectionEvt;
    cMessage* makeDBSCANEvt;
};

} // namespace veins
