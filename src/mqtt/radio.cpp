#include "radio.h"

Radio::Radio(MQTTClient * client){
    this->client = client;
}

void Radio::config(float wifiTransferSpeed_Mbps, float maxJitter_ms, float message_loss_probability){
    //
}

void Radio::broadcast_message(std::string &messagePrependedWithId) const{
    messagePrependedWithId.insert(0, "<A>"); //Prepend with A
    client->addMessageToSend(messagePrependedWithId);
}

void Radio::send_message(std::string &messagePrependedWithId, const std::string &id){
    messagePrependedWithId.insert(0, "<" + id + ">"); //Prepend with target id
    client->addMessageToSend(messagePrependedWithId);
}

void Radio::receive_messages(std::vector<std::string> &messages, double current_time_s){
    client->getIncomingMessages(messages);
}
