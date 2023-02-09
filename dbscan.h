#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <sstream>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;

typedef struct Point_
{
    double x, y, z;  // X, Y, Z position
    double speed = 0.0;    // Speed vehicle
    //double direction; // current direction
    char* roadId; //Road or Lane ID
    int selfID = -1; // Vehicle ID
    int clusterID;   // clustered ID
}Point;

class DBSCAN {
public:    
    DBSCAN(unsigned int minPts, float eps, vector<Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN(){}

    int run();
    vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(Point pointCore, Point pointTarget);
    inline bool verifyRoad(Point pointCore, Point pointTarget);
    inline bool verifyDirection(Point pointCore, Point pointTarget);

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}
    void reverseRoad(string& road);
    string getParallelRoad(string road);
    string findRoad(string road);
    vector<Point> m_points;
private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

#endif // DBSCAN_H
