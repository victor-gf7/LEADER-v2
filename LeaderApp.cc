#include "veins/modules/application/traci/LeaderApp.h"

using namespace veins;

Define_Module(veins::LeaderApp);


LeaderApp::~LeaderApp() {
    if(outputFile.is_open()){
        outputFile.close();
    }
    if(rawDataFile.is_open()){
        rawDataFile.close();
    }
    if(leadersFile.is_open()){
        leadersFile.close();
    }
//    if(plotClusters.is_open()){
//        plotClusters.close();
//    }
}

bool is_number(const std::string& s) {
    try {
        std::stoi(s);
    } catch (std::exception& e) {
        return false;
    }
    return true;
}

void LeaderApp::onWSA(DemoServiceAdvertisment* wsa){
}

void LeaderApp::onWSM(BaseFrame1609_4* frame){
}

std::map<int, float> LeaderApp::sort(map<int, float> map)
{
    set<pair<int, float>, comp> sortedSet(map.begin(), map.end());

    std::map<int,float> sortedMap;

    for (auto& it : sortedSet) {
        sortedMap.insert(pair<int, float>(it.first, it.second));
    }

    return sortedMap;
}

void LeaderApp::conductElection(std::map<int, float> map){
    //std::map<int,float> sortedMap = sort(map);
    leadersMap.clear();

    for (itClustermap = Clustermap.begin(); itClustermap != Clustermap.end(); ++itClustermap){
        if(itClustermap->first != -1){
            for(itList = itClustermap->second.begin(); itList != itClustermap->second.end(); ++itList){

                if(leadersMap.find(itClustermap->first) == leadersMap.end()){//verifica se existe algum cluster
                    leader.selfID.push_back(*itList);//caso não tenha insere independente
                    leader.value.push_back(map[*itList]);
                    leadersMap.insert(pair<int, Leader>(itClustermap->first, leader));
                } else{//caso exista o cluster no mapa
                    if(leadersMap[itClustermap->first].selfID.size() < LEADERS_QTD){//verifica se ja tem a quantidade de lideres sufucuente
                        leadersMap[itClustermap->first].selfID.push_back(*itList);//caso não, insere independente
                        leadersMap[itClustermap->first].value.push_back(map[*itList]);
                    } else{//se já tiver
                        //pega o indice do menor valor no vetor
                        int minElementIndex = std::min_element(
                                leadersMap[itClustermap->first].value.begin(),
                                leadersMap[itClustermap->first].value.end()) - leadersMap[itClustermap->first].value.begin();

                        for(int i = 0; i < LEADERS_QTD; i++){//percorre o vetor com os lideres

                            if(leadersMap[itClustermap->first].value[i] < map[*itList]){//verifica se o novo valor é maior do que o já presente
                                leadersMap[itClustermap->first].selfID.erase(
                                        leadersMap[itClustermap->first].selfID.begin()+minElementIndex);
                                leadersMap[itClustermap->first].value.erase(
                                        leadersMap[itClustermap->first].value.begin()+minElementIndex);

                                leadersMap[itClustermap->first].selfID.push_back(*itList);
                                leadersMap[itClustermap->first].value.push_back(map[*itList]);
                                break;
                            }

                        }
                    }

                }
                leader.selfID.clear();
                leader.value.clear();
            }
        }
    }

}

void LeaderApp::onBSM(DemoSafetyMessage* bsm){
    switch(bsm->getKind()){
    case SEND_BEACON_EVT:{
        posString = std::to_string(bsm->getSenderPos().x)+";"+std::to_string(bsm->getSenderPos().y)+";"+std::to_string(bsm->getSenderPos().z)+";"+std::to_string(bsm->getSenderSpeedDouble())+";"+bsm->getRoadID();
        messageCounter++;
        //Abrir arquivo!
        if(!counterFile.is_open()){
            counterFile.open("Resultados/Counter_Data.txt");
            counterFile << "actualTime;quantity" << "\n";
        }
        counterFile << simTime().dbl() << ";" << messageCounter << "\n";

        //std::cout << "posString: " << posString << endl;
        if(mapDBSCAN.find(bsm->getSourceAddress()) == mapDBSCAN.end()){
            mapDBSCAN.insert(std::pair<int,std::string>(bsm->getSourceAddress(),posString));
        }else{
            mapDBSCAN[bsm->getSourceAddress()] = posString;
        }

        //First Scheduling
        if(eventoEscalonado == 0){
            eventoEscalonado = 1;
            self = getParentModule()->getIndex(); // ID VERDADEIRO
            makeDBSCANEvt = new cMessage("Make DBSCAN Event", MAKE_DBSCAN_EVT);
            leaderElectionEvt = new cMessage("Leader Election Event", LEADER_ELECTION_EVT);
            windowTime = par("windowTime").doubleValue();
            strategy = par("strategy").stringValue();
            fileName = par("fileName").stringValue();
            scheduleAt(simTime().dbl()+windowTime, makeDBSCANEvt);
            sizeMap = -1;

            //Configure TRACI
            manager = TraCIScenarioManagerAccess().get();
            ASSERT(manager);
            traci = manager->getCommandInterface();
            if (!traci) {
                std::cout << "Cannot create screenshot: TraCI is not connected yet" << endl;
            }
            //Configure TRACI
        }
        //First Scheduling

        break;
    }
    default:{
        std::cout << "onBSM - The message type was not detected." << bsm->getKind() << endl;
        break;
    }
    }
}

// BFS algorithm is used to calculate all the single source shortest paths in a non weighted graph and the source's closeness.
float LeaderApp::bfs_SSSP(int src, int n, stack<int> &visitStack, vector<int> &sigma, list<int> *pred, const adjacency_list &adjList) {
    // Closeness counter.
    float closeness = 0;

    // Vector that holds the distances from the source.
    vector<int> dist;
    dist.resize(n, -1);
    dist[src] = 0;

    // Queue used for the Bfs algorithm.
    queue<int> visitQueue;
    visitQueue.push(src);
    //cout<<"debug 30"<<endl;
    // While there are still elements in the queue.
    while (!visitQueue.empty()) {
        // Pop the first.
        int v = visitQueue.front();
        visitQueue.pop();
        visitStack.push(v);

        // Closeness part aggregation.
        closeness += dist[v];

        // Check the neighbors w of v.
        //for (vector<int>::iterator it = adjList[v].begin(); it != adjList[v].end(); it++) {
        //cout<<"adjbfs_SSSP size : "<<adjList[v].size()<<"; this is v :"<<v<<endl;
        for(int i=0;i<adjList[v].size();i++){
            int w = adjList[v].at(i);
            // Node w found for the first time?
            /* DESCUBRA JOÃO
            if(v==62){
                cout<<"debug x"<<v<<" "<<w<<endl;
            }
            */
            if (dist[w] < 0) {
                visitQueue.push(w);
                dist[w] = dist[v] + 1;
            }

            // Is the shortest path to w via u?
            if (dist[w] == dist[v] + 1) {
                pred[w].push_back(v);
                sigma[w] += sigma[v];
            }

        }

    }
    // Closeness part inversion.
    if (closeness!=0) {
        return 1.0 / closeness;
    } else {
        return 0;
    }
}

void LeaderApp::resetVariables(int src, int n, list<int> *pred, vector<int> &sigma, vector<float> &delta) {
    for (int i = 0; i < n; i++) {
        pred[i].clear();
    }

    sigma.clear();
    sigma.resize(n, 0);
    sigma[src] = 1;

    delta.clear();
    delta.resize(n, 0);
}

void LeaderApp::handleSelfMsg(cMessage* msg){
    switch (msg->getKind()) {
    case MAKE_DBSCAN_EVT:{
        if(mapDBSCAN.empty()){
            std::cout << simTime().dbl() << " - Empty MAP! Do not do DBSCAN" << endl;
        }else{
            //Abrir arquivo!
            if(!outputFile.is_open()){
                outputFile.open("Resultados/DBSCAN_Data_"+fileName+"_"+std::to_string(self)+".txt");
                outputFile << "clusterID;selfID;x;y;speed;actualTime" << "\n";
            }
            if(!rawDataFile.is_open()){
                rawDataFile.open("Resultados/RAW_Data_"+fileName+"_"+std::to_string(self)+".txt");
            }
            if(!leadersFile.is_open()){
                leadersFile.open("Resultados/Leaders_data_"+fileName+"_"+std::to_string(self)+".txt");
                leadersFile << "clusterID;selfID;representativenessValue;actualTime" << "\n";
            }
//            if(!plotClusters.is_open()){
//                plotClusters.open("Resultados/plots_data_"+fileName+"_"+std::to_string(self)+".txt");
//            }
            //Abrir arquivo!

            actualTime = simTime().dbl();
            std::cout << "\nSIM_TIME: " << actualTime << endl;
            int tamanhoDBSCAN = mapDBSCAN.size();
            if(sizeMap < tamanhoDBSCAN){
                sizeMap = 901;
            }
            std::cout << "sizeMap: " << sizeMap << endl;

            //BETWEENESS INIT
            nodeBetweenness.resize(sizeMap, 0);
            closeness.resize(sizeMap, 0);
            random.resize(sizeMap,0);
            std::list<int> pred[sizeMap]; // List of predecessors of node v.

            e = 0;
            weight = 1;
            isWeigthed = false;
            adjList.reserve(sizeMap);
            adjList.resize(sizeMap);
            //BETWEENESS INIT

            //SEPARATE THE POINTS
            for (itmapDBSCAN = mapDBSCAN.begin(); itmapDBSCAN != mapDBSCAN.end(); ++itmapDBSCAN){
                //std::cout << itmapDBSCAN->first << " - handleSelfMsg " << itmapDBSCAN->second << " - SIM_TIME: " << simTime().dbl() <<   endl;
                positionByNode = DemoBaseApplLayer::split(itmapDBSCAN->second, ";");

                ponto.x = std::stod(positionByNode[0]);
                ponto.y = std::stod(positionByNode[1]);
                ponto.z = std::stod(positionByNode[2]);
                ponto.speed = std::stod(positionByNode[3]);
                ponto.roadId = strdup(positionByNode[4].c_str());
                //ponto.direction = std::stod(positionByNode[5]);
                ponto.selfID = itmapDBSCAN->first;
                ponto.clusterID = UNCLASSIFIED;
                points.insert(points.begin(), ponto);

                rawDataFile << "" << positionByNode[0]  <<
                              ";" << positionByNode[1] <<
                              ";" << positionByNode[2] <<
                              ";" << positionByNode[3] <<
                              ";" << positionByNode[4] <<
                              //";" << positionByNode[5] <<
                              ";" << ponto.selfID <<
                              ";" << actualTime <<
                              "\n";
            }
            //rawDataFile << "\n\n";
            //SEPARATE THE POINTS

            //Run the DBSCAN
            DBSCAN ds(MINIMUM_POINTS, EPSILON_DBSCAN, points);
            ds.run();
            //Run the DBSCAN

            //MOSTRAR RESULTADOS
            iDBSCAN = 0;
            pointsResult = ds.m_points;
            while(iDBSCAN < ds.getTotalPointSize()){
                if(pointsResult[iDBSCAN].clusterID != -1){
                    outputFile << "" << pointsResult[iDBSCAN].clusterID << //ID do cluster
                                  ";" << pointsResult[iDBSCAN].selfID << //ID do veículo
                                  ";" << pointsResult[iDBSCAN].x << //Posição X
                                  ";" << pointsResult[iDBSCAN].y << //Posição Y
                                  ";" << pointsResult[iDBSCAN].speed << //Velocidade
                                  //";" << pointsResult[iDBSCAN].direction << //direção
                                  ";" << actualTime << //Tempo atual de simulação
                                  "\n";

                }
                mapNodeByCluster.insert(std::pair<int, int>(pointsResult[iDBSCAN].selfID, pointsResult[iDBSCAN].clusterID));
                Clustermap[pointsResult[iDBSCAN].clusterID].push_back(pointsResult[iDBSCAN].selfID);
                iDBSCAN++;
            }
//            plotClusters << "\n\n";
            //MOSTRAR RESULTADOS

            //LEADER ELECTION
            std::cout << actualTime << " - ELECTION" << endl;
            for (itClustermap = Clustermap.begin(); itClustermap != Clustermap.end(); ++itClustermap){
                if(itClustermap->first != -1){
                    std::cout <<  actualTime << " - Membros do Cluster " << itClustermap->first << endl;
                    for(itList = itClustermap->second.begin(); itList != itClustermap->second.end(); ++itList){
                        positionVizinhoA = DemoBaseApplLayer::split(mapDBSCAN[*itList], ";");
                        std::cout << "Vizinhos de " << *itList << " POS X:" << std::stod(positionVizinhoA[0]) << " POS Y:" << std::stod(positionVizinhoA[1]) << " POS Z:" << std::stod(positionVizinhoA[2]) << endl;

                        for(jtList = itClustermap->second.begin(); jtList != itClustermap->second.end(); ++jtList){
                            if(*jtList != *itList){
                                positionVizinhoB = DemoBaseApplLayer::split(mapDBSCAN[*jtList], ";");
                                distAB = sqrt(pow((std::stod(positionVizinhoA[0]) - std::stod(positionVizinhoB[0])), 2) + pow((std::stod(positionVizinhoA[1]) - std::stod(positionVizinhoB[1])), 2));

                                //std::cout << "Esse cara eh meu vizinho? " << *jtList << " distAB: "<< distAB
                                //<< " POS X:" << std::stod(positionVizinhoB[0])
                                //<< " POS Y:" << std::stod(positionVizinhoB[1])
                                //<< " POS Z:" << std::stod(positionVizinhoB[2])
                                //<< endl;

                                if(distAB <= 20.0){
                                    //INSERIR VIZINHO NA LISTA DE ADJASCENCIA
                                    //std::cout << actualTime << " - YES!!" << *jtList << endl;
                                    e += 1;
                                    start = *itList;
                                    end = *jtList;
                                    adjList[start].push_back(end);
                                    adjList[end].push_back(start);
                                    //INSERIR VIZINHO NA LISTA DE ADJASCENCIA
                                }else{
                                    //std::cout << "NOT VIZINHO!" << *jtList << endl;
                                }
                            }
                        }
                        //Todos os vizinhos foram percorridos
                    }
                }
            }
//            if (!testeFile.is_open()) {
//                testeFile.open("Resultados/teste.txt");
//            }

            //VALIDATE STATEGY
            if(strategy != "none"){
                //BETWEENESS & CLOSENESS CALC
                srand( (unsigned)time( NULL ) );
                for (int src = 0; src < sizeMap; src++) {
                    // Prepare the variables for the next loop.
                    //adjacency_list adjList; // Adjacency list.
                    //readGraph(n, isWeigthed, adjList, edgeBetweenness);
                    resetVariables(src, sizeMap, pred, sigma, delta);
                    //cout<<"debug 22.1 : "<<src<<endl;
                    closeness[src] = bfs_SSSP(src, sizeMap, visitStack, sigma, pred, adjList);

                    while (!visitStack.empty()) {
                        int w = visitStack.top();
                        visitStack.pop();

                        // For each predecessors of node w, do the math!
                        for (list<int>::iterator it = pred[w].begin(); it != pred[w].end(); it++) {
                            int v = *it;
                            float c = ((float) sigma[v] / (float) sigma[w]) * (1.0 + delta[w]);

                            //testeFile << "\nSigma v: " << sigma[v] << "\nsigma w: " << sigma[w] << "\nc: " << c << endl;
                            if(!isnan(c) && !isinf(c) && c > 0){
                                delta[v] += c;
                            }

                        }
                        // Node betweenness aggregation part.
                        if (w != src) {
                            nodeBetweenness[w] += delta[w];
                        }
                        random[w] = (float) rand()/RAND_MAX;
                    }
                }
                //BETWEENESS & CLOSENESS CALC
                bool normalize = true;
                if(strategy == "closeness"){
                    normalize = false;
                    //CLOSENESS
                    int minValue = *min_element(closeness.begin(),
                            closeness.end());
                    int maxValue = *max_element(closeness.begin(),
                            closeness.end());
                    leadersMap.clear();
                    for (int i = 0; i < sizeMap; i++) {
                        if (normalize) {
                            double nomalizedValue = (closeness[i] - minValue) / (maxValue - minValue);
                            //conductElection(i, nomalizedValue);
                            closenessMap.insert(std::pair<int, float>(i, nomalizedValue));
                            std::cout << "CENTRALITY - Node " << i << ": "
                                    << nomalizedValue << endl;
                        } else {
                            //conductElection(i, closeness[i]);
                            closenessMap.insert(std::pair<int, float>(i, closeness[i]));
                            std::cout << "CENTRALITY - Node " << i << ": "
                                    << closeness[i] << endl;
                        }
                    }
                    conductElection(closenessMap);
                    //END CLOSENESS
                } else if(strategy == "betweenness"){
                    //normalize = false;
                    //BETWEENESS
                    //nrml = 1;
                    //if (normalize) {
                    //  nrml = (sizeMap - 1)*(sizeMap - 2);
                    //}
                    int minValueBw = *min_element(nodeBetweenness.begin(),
                            nodeBetweenness.end());
                    int maxValueBw = *max_element(nodeBetweenness.begin(),
                            nodeBetweenness.end());
                    leadersMap.clear();
                    for (int i = 0; i < sizeMap; i++) {
                        if (normalize) {
                            double nomalizedValueBw = (nodeBetweenness[i] - minValueBw) / (maxValueBw - minValueBw);
                            if(isnan(nomalizedValueBw)){
                                nomalizedValueBw = 0;
                            }
                            //conductElection(i, nomalizedValueBw);
                            nodeBetweennessMap.insert(std::pair<int, float>(i, nomalizedValueBw));
                            std::cout << "BETWEENESS - Node " << i << ": "
                                    << nomalizedValueBw << endl;
                        } else {
                            //conductElection(i, nodeBetweenness[i]);
                            nodeBetweennessMap.insert(std::pair<int, float>(i, nodeBetweenness[i]));
                            cout << "BETWEENESS - Node " << i << ": "
                                    << nodeBetweenness[i] << endl;
                        }
                    }
                    conductElection(nodeBetweennessMap);
                    //END BETWEENESS
                } else if(strategy == "random"){
                    leadersMap.clear();
                    for(int i = 0; i < sizeMap ; i++){
                        //conductElection(i, random[i]);
                        randomMap.insert(std::pair<int, float>(i, random[i]));
                        cout << "RANDOM - Node " << i << ": "
                                << random[i] << endl;
                    }
                    conductElection(randomMap);
                }
            }
            //END VALIDADE STRTEGY

            leadersMap.erase(-1);
            leadersMap.erase(0);
            int index = 0;
            for(itLeadersMap = leadersMap.begin(); itLeadersMap != leadersMap.end(); ++itLeadersMap){
                if(itLeadersMap->first != -1){

                    for(int i = 0; i < LEADERS_QTD; i++){
                        leadersFile << itLeadersMap->first << ";"
                                << itLeadersMap->second.selfID[i] << ";"
                                << itLeadersMap->second.value[i] << ";"
                                << actualTime << "\n";

                        std::cout << "ClusterID: " << itLeadersMap->first <<
                                " Leader: " << itLeadersMap->second.selfID[i] << " value: "<<
                                itLeadersMap->second.value[i]  << endl;
                    }

                }
            }
            //LEADER ELECTION

            //BETWEENESS CLEANER
            adjList.clear();
            //BETWEENESS CLEANER
        }

        points.clear();
        pointsResult.clear();
        mapDBSCAN.clear();
        Clustermap.clear();
        mapNodeByCluster.clear();
        closenessMap.clear();
        nodeBetweennessMap.clear();
        randomMap.clear();

        scheduleAt(simTime().dbl() + windowTime, makeDBSCANEvt);
        scheduleAt(simTime().dbl() + uniform(0.0,1), leaderElectionEvt);
        break;
    }
    case LEADER_ELECTION_EVT: {
        std::cout << "Evento de leader executado: " << simTime().dbl() << endl;
        if(!leadersMap.empty()){
            DemoSafetyMessage* bsm = new DemoSafetyMessage();
            DemoBaseApplLayer::populateWSM(bsm);

            bsm->setSourceAddress(self);
            bsm->setTimestamp(simTime());
            bsm->setKind(LEADER_ELECTION_EVT);
            bsm->setLeadersArraySize(leadersMap.size() * LEADERS_QTD);

            int index = 0;
            for(itLeadersMap = leadersMap.begin(); itLeadersMap != leadersMap.end(); ++itLeadersMap){
                if(itLeadersMap->first != -1){
                    for(int i = 0; i < LEADERS_QTD; i++){
                        bsm->setLeaders(index,itLeadersMap->second.selfID[i]);
                        index++;
                    }

                }
            }

            DemoBaseApplLayer::sendDelayedDown(bsm, uniform(0.0,1));
            std::cout << "Evento de leader enviado: " << simTime().dbl() << endl;
            break;
        }
        break;
    }
    default: {
        EV << "handleSelfMsg - The message type was not detected." << endl;
        break;
    }
    }
}
