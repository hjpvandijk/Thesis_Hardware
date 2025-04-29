/*
 * TestTrans.h
 *
 *  Created on: 4 Sept 2022
 *      Author: jondurrant
 */

#ifndef MQTTCLIENT_H_
#define MQTTCLIENT_H_


#include "../TaskAgent.h"


#include <vector>
#include <string>


#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


class MQTTClient : public TaskAgent{
public:
	MQTTClient(std::string id);
	virtual ~MQTTClient();
    void addMessageToSend(std::string message);
    void getIncomingMessages(std::vector<std::string> &messages);
	bool isConnected();



protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

	void test();
    
    mqtt_client_t *client;

private:
    std::string id;
    //Our client

    //Messages to send
    static QueueHandle_t mqttSendQueue;

    //Incoming messages
    static QueueHandle_t mqttIncomingQueue;
    static std::string incomingBuffer;

    void sendMessages();

	bool isConnected(mqtt_client_t *client);


	bool testConnect();
	bool testTrans();
	bool tryConnect(mqtt_client_t *client);
	void listenForMessages(mqtt_client_t *client);
	bool testTLS();
    static void mqtt_pub_request_cb(void *arg, err_t result);
	void publish_message_internal(mqtt_client_t *client, void *arg);
    static void publish_message_static_wrapper(void *arg);
    void publish_message(mqtt_client_t *client, const char* pub_payload);
	static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
    static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
    static void mqtt_sub_request_cb(void *arg, err_t result);
    static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
    static void dns_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);







	int xTests = 0;
	int xSuccessful = 0;
};

#endif /* TEST_TESTTRANS_H_ */