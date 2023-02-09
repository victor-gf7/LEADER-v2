#include "dbscan.h"

int DBSCAN::run(){
    int clusterID = 1;
    vector<Point>::iterator iter;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter){
        if (iter->clusterID == UNCLASSIFIED){
            if (expandCluster(*iter, clusterID) != FAILURE){
                clusterID += 1;
            }
        }
    }
    return 0;
}

int DBSCAN::expandCluster(Point point, int clusterID){
    vector<int> clusterSeeds = calculateCluster(point);
    if(clusterSeeds.size() < m_minPoints){
        point.clusterID = NOISE;
        return FAILURE;
    }
    else{
        int index = 0, indexCorePoint = 0;
        vector<int>::iterator iterSeeds;
        for( iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds){
            m_points.at(*iterSeeds).clusterID = clusterID;
            if(m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y && m_points.at(*iterSeeds).z == point.z){
                indexCorePoint = index;
            }
            ++index;
        }
        clusterSeeds.erase(clusterSeeds.begin()+indexCorePoint);
        for(vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i){
            vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));
            if(clusterNeighors.size() >= m_minPoints){
                vector<int>::iterator iterNeighors;
                for(iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors){
                    if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE )
                    {
                        if ( m_points.at(*iterNeighors).clusterID == UNCLASSIFIED )
                        {
                            clusterSeeds.push_back(*iterNeighors);
                            n = clusterSeeds.size();
                        }
                        m_points.at(*iterNeighors).clusterID = clusterID;
                    }
                }
            }
        }
        return SUCCESS;
    }
}

vector<int> DBSCAN::calculateCluster(Point point){
    int index = 0;
    vector<Point>::iterator iter;
    vector<int> clusterIndex;
    for(iter = m_points.begin(); iter != m_points.end(); ++iter){
        if(calculateDistance(point, *iter) <= m_epsilon /*&& verifyRoad(point, *iter)*/){
            clusterIndex.push_back(index);
        }
        index++;
    }
    return clusterIndex;
}

inline double DBSCAN::calculateDistance( Point pointCore, Point pointTarget){
    return sqrt(pow(pointCore.x - pointTarget.x,2)+pow(pointCore.y - pointTarget.y,2)+pow(pointCore.z - pointTarget.z,2));
}
void DBSCAN::reverseRoad(string& road){
    int n = road.length()-1;
    int i = 0;

    if(n<=i || road.length() != 4){return;}

    swap(road[i],road[i+2]);
    swap(road[n],road[n-2]);
}

string DBSCAN::getParallelRoad(string road){
//    string r = road.substr(road.find("-") + 1);
//    string j = r.substr(0, r.find("#"));
    return road.substr(0, road.find("_"));
}

string DBSCAN::findRoad(string road){
    std::stringstream finalRoad;
    if(road[0] == road[5]){
        finalRoad << road[0] << road[5];
    } else{
        finalRoad << road[2] << road[7];
    }

    return finalRoad.str();
}

inline bool DBSCAN::verifyDirection(Point pointCore, Point pointTarget){
    return true;
    //return pointCore.direction == pointTarget.direction;
}

inline bool DBSCAN::verifyRoad(Point pointCore, Point pointTarget){
//    std::string strTargetRoad(pointTarget.roadId);
//    std::string strCoreRoad(pointCore.roadId);
//    reverseRoad(strCoreRoad);
//
//    if(getParallelRoad(pointCore.roadId) == getParallelRoad(pointTarget.roadId)){
//        return true;
//    }else{
//        string roadTarget = findRoad(strTargetRoad);
//        string roadCore = findRoad(strCoreRoad);
//        return roadCore == roadTarget;
//    }

    return getParallelRoad(pointCore.roadId) == getParallelRoad(pointTarget.roadId);
}

