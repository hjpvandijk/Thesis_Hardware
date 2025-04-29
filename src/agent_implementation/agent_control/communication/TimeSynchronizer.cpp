#include "TimeSynchronizer.h"
#include "../../agent.h"


/**
 * Initiate the time synchronization by broadcasting our current time
 * @param initiating_agent
 */
void TimeSynchronizer::initTimeSync(Agent* initiating_agent){
    syncMissionTime(initiating_agent);

//    std::vector<std::string> toErase = {};
//    for (auto & agentSync : agentSyncs) {
//        if (initiating_agent->elapsed_ticks - std::get<0>(agentSync.second) >= initiating_agent->ticks_per_second * initiating_agent->config.TIME_SYNC_INTERVAL_S/2) {
//            //If we have already initiated a sync with this agent, and it has not responded in TIME_SYNC_INTERVAL_S/2 seconds, delete the sync
//            toErase.push_back(agentSync.first);
//        }
//    }
//    for (const std::string& agent_id : toErase) {
//        agentSyncs.erase(agent_id);
//    }
//    if (agentSyncs.empty()) {
        //determine t_txi
        int t_TXi = initiating_agent->elapsed_ticks;
        //Broadcast t_txi
        initiating_agent->broadcastMessage(t_TXiMessage(t_TXi));
//    lastSyncAttempts[other_agent_id] = agent->elapsed_ticks; //Store the time we have synced with this agent
        lastSyncAttemptTick = initiating_agent->elapsed_ticks;
//    }

}


/**
 * Received t_txi, determine t_rxj and t_txj, and send t_txi, t_rxj, t_txj
 * t_txi is the send time of the other agent
 */
void TimeSynchronizer::respondToTimeSync(std::string sender_id, Agent *receiving_agent, int t_TXi) {
//    if (agentSyncs.empty()) {
        if (agentSyncs.find(sender_id) == agentSyncs.end()) { //If we have no active sync with this agent
            //Insert the new sync, and set the t_TXi and t_RXj
            int t_RXj = receiving_agent->elapsed_ticks;
            int t_TXj = t_RXj;
            agentSyncs.insert({sender_id, {t_TXj, 0, t_TXi, t_RXj}}); //Here, __j is ours, __i is the other agent's
            //Send back t_txi, t_rxj, t_txj
            //Also sending back t_txi so they know to which of their messages we responded.
            receiving_agent->sendMessage(t_TXi_t_RXj_t_TXjMessage(t_TXi, t_RXj, t_TXj), sender_id);

        }
//    }
}

/**
 * Receive t_txi, t_rxj, t_txj, determine and send t_rxi
 * Then sync the mission time
 */
void TimeSynchronizer::determineT_RXi(std::string sender_id, Agent* receiving_agent, int t_TXi, int t_RXj, int t_TXj) {
//    if (receiving_agent->elapsed_ticks - this->lastSuccesfulSyncTick < receiving_agent->ticks_per_second * receiving_agent->config.TIME_SYNC_INTERVAL_S) {
//        return; //If we have synced in the last TIME_SYNC_INTERVAL_S seconds, we ignore this sync
//    }
    //Check if t_TXi is the earliest of all syncs
////    for (const auto& agentSync : agentSyncs) {
////        if (t_TXi >= std::get<0>(agentSync.second)) {
////            return;
////        }
////    }
//    //Delete all others
//    std::vector<std::string> toErase = {};
//    for (const auto& agentSync : agentSyncs) {
//        if (agentSync.first != sender_id) toErase.push_back(agentSync.first);
//    }
//    for (const std::string& agent_id : toErase) {
//        agentSyncs.erase(agent_id);
//    }

    if (agentSyncs.find(sender_id) == agentSyncs.end()) { //If we have no active sync with this agent
        //Determine t_RXi
        uint32_t t_RXi = receiving_agent->elapsed_ticks;

        agentSyncs.insert({sender_id, {t_TXi, t_RXj, t_TXj, t_RXi}}); //Here __i is ours, __j is the other agent's

        //Send t_rxi
        receiving_agent->sendMessage(t_RXiMessage(t_RXi), sender_id);

        calculateCompensation(sender_id, receiving_agent); //We have all the information to sync the mission time
    } else {
        //Highest ID wins the init, so if our ID is higher, we ignore this sync response
        if (receiving_agent->id > sender_id) {
            return;
        } else { //Else we erase the old sync (ours) and continue with this sync
            agentSyncs.erase(sender_id);
            uint32_t t_RXi = receiving_agent->elapsed_ticks;
            agentSyncs.insert(
                    {sender_id, {t_TXi, t_RXj, t_TXj, t_RXi}}); //Here __i is ours, __j is the other agent's

            //Send t_rxi
            receiving_agent->sendMessage(t_RXiMessage(t_RXi), sender_id);

            calculateCompensation(sender_id, receiving_agent); //We have all the information to sync the mission time
        }

    }
}

/**
 * Receive t_rxi
 * Then sync the mission time
 */
void TimeSynchronizer::receiveT_RXi(const std::string& sender_id, Agent* receiving_agent, int t_RXi){
    if (agentSyncs.find(sender_id) != agentSyncs.end()) { //If we have an active sync with this agent
        //Update the sync
        std::get<1>(agentSyncs[sender_id]) = t_RXi; //Here __j is ours, __i is the other agent's
    }

    calculateCompensation(sender_id, receiving_agent); //We have all the information to sync the mission time
}

/**
 * Sync the mission time with the other agent by calculating the time offset and compensating for it
 * @param other_agent_id
 * @param agent
 */
void TimeSynchronizer::calculateCompensation(const std::string& other_agent_id, Agent* agent) {
    if (agentSyncs.find(other_agent_id) == agentSyncs.end()) {
        return; //If we have no active sync with this agent, we ignore this sync
    }
    auto [t_TXi, t_RXj, t_TXj, t_RXi] = agentSyncs[other_agent_id];
    double time_offset = ((t_RXj - t_TXi) - (t_RXi - t_TXj)) / 2; //The difference in ticks between the agents
    int agent_compensation = std::floor(time_offset /
                                        2.0);//The amount this agent should compensate. Floor makes sure we get an integer difference, i.e. 0.5 means only one agent shifts one tick
    // assert(agent_compensation == 0);
    printf("Agent %s compensated %d ticks with agent %s\n", agent->id.c_str(), agent_compensation, other_agent_id.c_str());
    printf("Agent %s: t_TXi: %f t_RXj: %f t_TXj: %f t_RXi: %f\n", agent->id.c_str(), t_TXi, t_RXj, t_TXj, t_RXi);
//    if (agent_compensation != 0) {
        this->compensations[other_agent_id] = agent_compensation;

//        argos::LOG << "t_TXi: " << t_TXi << " t_RXj: " << t_RXj << " t_TXj: " << t_TXj << " t_RXi: " << t_RXi
//                   << std::endl;
//        argos::LOG << "Agent " << agent->id << " compensated " << agent_compensation << " ticks with agent "
//                   << other_agent_id << std::endl;
//    }

    agentSyncs.erase(other_agent_id); //Remove the active sync, because we have synced the time
    //Negative means agent is ahead of other agent
    //Positive means agent is behind other agent
}
void TimeSynchronizer::syncMissionTime(Agent* agent){
    printf("n compensations: %d\n", this->compensations.size());
    if (this->compensations.empty()) {
        return;
    }
    int agent_compensation = 0;
    for (auto& compensation : this->compensations){
        auto val = compensation.second;
        agent_compensation += val;
    }
    auto avg_compensation = static_cast<double>(agent_compensation) / this->compensations.size();
    agent_compensation = std::floor(avg_compensation);
    printf("Agent %s ticks before compensation: %d\n", agent->id.c_str(), agent->elapsed_ticks);
    uint32_t synced_ticks = agent->elapsed_ticks + agent_compensation;
    agent->elapsed_ticks = synced_ticks;
    printf("Agent %s ticks after compensation: %d\n", agent->id.c_str(), agent->elapsed_ticks);

//    argos::LOG << "Agent " << agent->id << " compensated average " << agent_compensation << " ticks" << std::endl;
    this->compensations.clear();
//    this->lastSuccesfulSyncTick = agent->elapsed_ticks;
//    agentSyncs.clear();//Remove all active syncs, because we have synced the time
}

std::string TimeSynchronizer::t_TXiMessage(double t_TXi) {
    return "T:0|" + std::to_string(t_TXi);
}

std::string TimeSynchronizer::t_TXi_t_RXj_t_TXjMessage(double t_TXi, double t_RXj, double t_TXj) {
    return "T:1|" + std::to_string(t_TXi) + "|" + std::to_string(t_RXj) + "|" + std::to_string(t_TXj);
}

std::string TimeSynchronizer::t_RXiMessage(double t_RXi) {
    return "T:2|" + std::to_string(t_RXi);
}


double TimeSynchronizer::getLastSyncAttempt(){
    return lastSyncAttemptTick;
}


