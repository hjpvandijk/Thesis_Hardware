#include "MicroControllerBatteryManager.h"
// #include "agent.h"
#include "../../agent.h"

/**
 * Estimate the power usage of the CPU for the given amount of seconds by estimating transmitting power, receiving power and idle power
 * @param agent
 * @param seconds
 * @return
 */
float MicroControllerBatteryManager::estimateCommunicationConsumption(Agent* agent, float seconds) const {
    auto [transmitPower, transmitTimeS] = estimateTransmitConsumption(agent, seconds);
    auto [receivePower, receiveTimeS] = estimateReceiveConsumption(agent, seconds);

    //Calculate the time we are not using RF
    float idleTimeS = seconds - transmitTimeS - receiveTimeS;
    float idlePowerUsage_mAh = idleTimeS * this->regularOperation_250MHz_mA / 3600.0f; //In mAh

    return transmitPower + receivePower + idlePowerUsage_mAh;
}

std::pair<float, float> MicroControllerBatteryManager::estimateTransmitConsumption(Agent* agent, float seconds) const{

    //Predict communication:
    //  - How many agents do we think there are
    //  - Have we already sent a message recently to them?



    //Translate into power usage
    //Calculate the size of the message to send
    //We know the amount of bits per node used
    //We know how many nodes per message
    //We know how many nodes in quadtree
    //So we can calculate how many messages
    //So we can use overhead to calculate the total size in bits of the messages
    //Then we can calculate how long transmitting takes
    //Then we can calculate how much power we used

    //Check if we have sent a message to this agent recently

    int nExchangeIntervalsInPeriod = std::floor( seconds/agent->config.QUADTREE_EXCHANGE_INTERVAL_S); //How many full exchange periods fit into the period
    double remaining = seconds - nExchangeIntervalsInPeriod * agent->config.QUADTREE_EXCHANGE_INTERVAL_S;

    //If the agent has sent a message to this agent recently, we will probably send a message soon
    //Else we will probably not send a message soon
    //So heuristics based on previous exchanges

    int amountOfTransmits = 0;

    for (auto &agentQuadtree : agent->agentQuadtreeSent) {
        if(agentQuadtree.second - agent->elapsed_ticks <= agent->config.QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second) { //If we have sent a message recently
            amountOfTransmits += nExchangeIntervalsInPeriod; //We will exchange nExchangeIntervalsInPeriod times with this agent
            if (agentQuadtree.second - agent->elapsed_ticks +
                agent->config.QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second <= remaining) { //If we will exchange soon
                amountOfTransmits++; //We are actually exchanging once more.
            }
        }


    }

    //Amount of nodes to exchange
    int nNodes = agent->quadtree->numberOfLeafNodes; //It is unknown how many nodes will be added in next 'seconds' so we will use the current amount of nodes
    //Calculage amount of messages we will send
    int nNodesPerMessage = agent->quadtree->numberOfNodesPerMessage;
    int nMessages = std::floor(nNodes / nNodesPerMessage);
    int remainingNodes = nNodes - nMessages * nNodesPerMessage;
    //Calculate size of messages
    int messageSize = nNodesPerMessage * this->bytesPerNode + this->targetSenderIDBytes;
    int remainingMessageSize = remainingNodes * this->bytesPerNode + this->targetSenderIDBytes;

    int totalNumberOfSentBytes = nMessages * messageSize + remainingMessageSize;
    //Calculate time to send all messages in a single exchange using the bandwidth
    float timeToTransmitS = float(totalNumberOfSentBytes) * 8.0f / (this->wifiTransferSpeed_Mbps * 1000000.0f);

    //Calculate power usage
    float quadtreeExchangePowerUsage_mAh = timeToTransmitS * this->wifiTransmitConsumption_mA / 3600.0f; //In mAh

    //Calculate total power usage, for all transmits within the given period
    float totalPowerUsage_mAh = quadtreeExchangePowerUsage_mAh * float(amountOfTransmits);

    //Return the total power usage and the time we spend transmitting within the period
    return {totalPowerUsage_mAh, timeToTransmitS*float(amountOfTransmits)};
}

std::pair<float, float> MicroControllerBatteryManager::estimateReceiveConsumption(Agent* agent, float seconds) const{

    float totalReceivePowerUsage_mAh = 0;
    float totalReceiveTimeS = 0;

    for (auto &agentQuadtree : agent->agentLocations) { // We assume agent location is received in (roughly) the same tick as the quadtree messages
        int nExchangeIntervalsInPeriod = std::floor( seconds/agent->config.QUADTREE_EXCHANGE_INTERVAL_S);
        double remaining = seconds - nExchangeIntervalsInPeriod * agent->config.QUADTREE_EXCHANGE_INTERVAL_S;

        int amountOfReceives = 0;
        if(std::get<2>(agentQuadtree.second) - agent->elapsed_ticks <= agent->config.QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second) { //If we have received a message from this agent recently
            amountOfReceives += nExchangeIntervalsInPeriod; //We will exchange nExchangeIntervalsInPeriod times with this agent
            if (std::get<2>(agentQuadtree.second) - agent->elapsed_ticks +
                agent->config.QUADTREE_EXCHANGE_INTERVAL_S * agent->ticks_per_second <= remaining) { //If we will (probably) receive soon
                amountOfReceives++; //We are actually exchanging once more.
            }
        }

        int previousNBytesReceived = agent->agentQuadtreeBytesReceived[agentQuadtree.first]; //Amount of bytes we received from this agent previously (we use this for the calculation)
        float timeToReceiveS = float(previousNBytesReceived) * 8.0f / (this->wifiTransferSpeed_Mbps * 1000000.0f); //Time to receive the message
        float quadtreeExchangePowerUsage_mAh = timeToReceiveS * this->wifiReceiveConsumption_mA / 3600.0f; //In mAh, Power usage to receive the message
        totalReceivePowerUsage_mAh += quadtreeExchangePowerUsage_mAh * float(amountOfReceives); //Power usage to receive all the quadtree messages from this agent added to the total
        totalReceiveTimeS += timeToReceiveS * float(amountOfReceives); //Time to receive all the quadtree messages from this agent added to the total
    }

    return {totalReceivePowerUsage_mAh, totalReceiveTimeS};

}
