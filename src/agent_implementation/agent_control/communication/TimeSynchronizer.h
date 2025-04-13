
#ifndef IMPLEMENTATION_AND_EXAMPLES_TIMESYNCHRONIZER_H
#define IMPLEMENTATION_AND_EXAMPLES_TIMESYNCHRONIZER_H

#include <map>
#include <vector>
#include <cstdio>
#include <string>

class Agent;

//Based on the paper: https://arxiv.org/pdf/2405.18384
class TimeSynchronizer {
public:
    TimeSynchronizer() = default;


    void initTimeSync(Agent* initiating_agent);

    //Received t_txi, determine t_rxj and send t_txi, t_txj, t_rxj
    void respondToTimeSync(std::string sender_id, Agent* receiving_agent, int t_TXi);

    //Receive t_rxi, t_rxj, t_txj, determine and send t_rxi
    void determineT_RXi(std::string sender_id, Agent* receiving_agent, int t_TXi, int t_RXj, int t_TXj);

    //Receive t_rxi
    void receiveT_RXi(const std::string& sender_id, Agent* receiving_agent, int t_RXi);

    void calculateCompensation(const std::string &other_agent_id, Agent *agent);

    void syncMissionTime(Agent* agent);

    double getLastSyncAttempt();

private:
    //Make sure i is always own, and j is always other
    std::map<std::string, std::tuple<double, double, double, double>> agentSyncs; //id: ( t_TXi, t_RXj, tTXj, tRXi )
    int lastSyncAttemptTick = 0;
    int lastSuccesfulSyncTick = 0;
//    std::vector<int> compensations = {};
    std::map<std::string, int> compensations = {};

    std::string t_TXiMessage(double t_TXi);
    std::string t_TXi_t_RXj_t_TXjMessage(double t_TXi, double t_RXj, double t_TXj);
    std::string t_RXiMessage(double t_RXi);

};


#endif //IMPLEMENTATION_AND_EXAMPLES_TIMESYNCHRONIZER_H
