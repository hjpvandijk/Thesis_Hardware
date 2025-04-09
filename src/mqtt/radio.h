//
// Created by hugo on 18-6-24.
//

#ifndef ARGOS3_EXAMPLES_RADIO_H
#define ARGOS3_EXAMPLES_RADIO_H

#include <queue>
#include <string>
#include "MQTTClient.h" // Include the header file where MQTTClient is defined


class Radio {
public:

    Radio() = default;
    Radio(MQTTClient * client);


    void config(float wifiTransferSpeed_Mbps, float maxJitter_ms, float message_loss_probability);

    void broadcast_message(std::string &messagePrependedWithId) const;
    void send_message(std::string &messagePrependedWithId, const std::string& id);

    void receive_messages(std::vector<std::string> &messages, double current_time_s);

private:
   MQTTClient *client;


};


#endif //ARGOS3_EXAMPLES_RADIO_H
