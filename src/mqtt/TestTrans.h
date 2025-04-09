/*
 * TestTrans.h
 *
 *  Created on: 4 Sept 2022
 *      Author: jondurrant
 */

#ifndef TEST_TESTTRANS_H_
#define TEST_TESTTRANS_H_

#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include "Agent.h"
#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"

class TestTrans : public Agent{
public:
	TestTrans();
	virtual ~TestTrans();

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

private:

	bool isConnected(mqtt_client_t *client);


	bool testConnect();
	bool testTrans();
	bool tryConnect(mqtt_client_t *client);
	void listenForMessages(mqtt_client_t *client);
	bool testTLS();
    static void mqtt_pub_request_cb(void *arg, err_t result);
    static void example_publish(mqtt_client_t *client, void *arg);
    static void mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len);
    static void mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
    static void mqtt_sub_request_cb(void *arg, err_t result);
    static void example_do_connect(mqtt_client_t *client);
    static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
    static void dns_callback(const char *name, const ip_addr_t *ipaddr, void *callback_arg);







	int xTests = 0;
	int xSuccessful = 0;
};

#endif /* TEST_TESTTRANS_H_ */