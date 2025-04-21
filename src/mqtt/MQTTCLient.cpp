/*
 * MQTTClient.cpp
 *
 *  Created on: 4 Sept 2022
 *      Author: jondurrant
 */

#include "MQTTClient.h"
#include "../WifiHelper.h"
// #include "TCPTransport.h"
// #include "TLSTransBlock.h"
#include "../config.h"
// #include "transport_interface.h"
#include <memory>
#include <semphr.h>
#include <cstring>
#include "lwip/stats.h"
#include "lwip/tcp.h"

// Define the PublishContext structure
struct PublishContext {
    char* payload;
};


// Define the MQTT connection parameters
// #define MQTT_BROKER_HOST "io.adafruit.com"
#define MQTT_CLIENT_ID "TestClient"
#define MQTT_TOPIC "test/topic"
#define MQTT_MESSAGE "Hello, MQTT"
ip_addr_t broker_ip;
SemaphoreHandle_t xSemaphoreDNSReady = NULL;
QueueHandle_t MQTTClient::mqttSendQueue = NULL; 
QueueHandle_t MQTTClient::mqttIncomingQueue = NULL; // Initialize the static queue
std::string MQTTClient::incomingBuffer;

bool mqtt_ready_to_send = true;
volatile int inflight_messages = 0;
int ticks_not_ready = 0;

int sentMessages = 0;

MQTTClient::MQTTClient(std::string id) {
    this->id = id;
	//Create queues
    mqttSendQueue = xQueueCreate(50, sizeof(const char *));
    if (mqttSendQueue == NULL) {
        printf("Failed to create send queue\n");
    }
    mqttIncomingQueue = xQueueCreate(50, sizeof(std::string*));
    if (mqttIncomingQueue == NULL) {
        printf("Failed to create incoming queue\n");
    }

}

MQTTClient::~MQTTClient() {
	// TODO Auto-generated destructor stub
}



void MQTTClient::run() {
    client = mqtt_client_new();
    if (client == NULL) {
        printf("Failed to create MQTT client\n");
        return;
    }

    // Initial connection
    tryConnect(client);

    //Last wake time
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (true) {
        // printf("mqtt running on core %d\n", get_core_num());    
        if (!isConnected(client)) {
            printf("Disconnected. Reconnecting...\n");
            tryConnect(client);
            //Wait a bit more
            vTaskDelay(1000 / 10);
        }else {
            // printf("Connected. Sending messages...\n");
            sendMessages(); // Send messages from the queue if connected
            // printf("Sent messages\n");
        }
        // vTaskDelay((1000/50)/portTICK_PERIOD_MS);  // Wait for 2 seconds before checking again
        vTaskDelayUntil(&xLastWakeTime, (1000/50) / portTICK_PERIOD_MS);

    }
}



/***
* Get the static depth required in words
* @return - words
*/
configSTACK_DEPTH_TYPE MQTTClient::getMaxStackSize(){
	return 5000;
}



/* Called when publish is complete either with success or failure */
void MQTTClient::mqtt_pub_request_cb(void *arg, err_t result)
{
  inflight_messages--;
  if(result != ERR_OK) {
    printf("Publish result from cb: %d\n", result);
    mqtt_ready_to_send = false;
  } else {
    // printf("Publish ok from cb\n");
    mqtt_ready_to_send = true;
    if (arg != NULL) {
        free((char*)arg); // Free the allocated memory after sending, arg should be the payload
    }
    
  }
}

void MQTTClient::publish_message(mqtt_client_t *client, void *arg, const char* pub_payload)
{
  err_t err;
  u8_t qos = 0; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain payload... */
//   printf("Publishing message: %s\n", pub_payload);
//   printf("Publishing to topic %s\n", IO_USERNAME "/feeds/" IO_FEED_NAME);
//   err = mqtt_publish(client, IO_USERNAME "/feeds/" IO_FEED_NAME, pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
//     printf("TCP seg used: %d / %d : %d\n", lwip_stats.memp[MEMP_TCP_SEG]->used, lwip_stats.memp[MEMP_TCP_SEG]->max, lwip_stats.memp[MEMP_TCP_SEG]->avail);
//   printf("MEMP_NUM_TCP_SEG: %d\n", MEMP_NUM_TCP_SEG);
//     printf("TCP Retransmission errors: %d\n", lwip_stats.tcp.rterr);
//   printf("Publishing to topic %s\n", MQTT_TOPIC);
  err = mqtt_publish(client, MQTT_TOPIC, pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, (void*) pub_payload);
//   printf("Sent message\n");
  inflight_messages++;
  sentMessages++;
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
    inflight_messages--;
    mqtt_ready_to_send = false;
    free((char*)pub_payload); // Free the allocated memory after sending
  }
//   else {
//     printf("Publish ok\n");
//   }
//   extern struct stats_ lwip_stats;
//   vTaskDelay(10);



}


/* The idea is to demultiplex topic and create some reference to be used in data callbacks
   Example here uses a global variable, better would be to use a member in arg
   If RAM and CPU budget allows it, the easiest implementation might be to just take a copy of
   the topic string and use it in mqtt_incoming_data_cb
*/
// static int inpub_id;
void MQTTClient::mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
//   printf("Incoming publish at topic %s with total length %u\n", topic, (unsigned int)tot_len);

//   /* Decode topic string into a user defined reference */
//   if(strcmp(topic, IO_USERNAME "/feeds/" IO_FEED_NAME) == 0) {
//     inpub_id = 0;
//   } else if(topic[0] == 'A') {
//     /* All topics starting with 'A' might be handled at the same way */
//     inpub_id = 1;
//   } else {
//     /* For all other topics */
//     inpub_id = 2;
//   }
}

void MQTTClient::mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags) {
    // printf("length: %d\n", len);
    // printf("Incoming data: ");
    // fwrite(data, 1, len, stdout);
    // fflush(stdout);
    // printf("\n");
    if (data == nullptr || len == 0) return;
    // Append chunk to the buffer
    incomingBuffer.append(reinterpret_cast<const char*>(data), len);

    // Check if this is the final chunk
    if (flags & MQTT_DATA_FLAG_LAST) {
        // printf("Received full message: %s\n", incomingBuffer.c_str());
        // printf("Received full message\n");

        // Make a heap-allocated copy and enqueue
        std::string* msgCopy = new std::string(incomingBuffer);
        // printf("Enqueuing message: %s\n", msgCopy->c_str());
        if (xQueueSend(mqttIncomingQueue, &msgCopy, 0) != pdTRUE) {
            // printf("Failed to send incoming message to queue\n");
            delete msgCopy;
        }
        // printf("Enqueued message\n");
        // Clear buffer for next message
        incomingBuffer.clear();
    } 
    // else {
    //     printf("Received chunk of data, not final\n");
    // }
}

void MQTTClient::mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n", result);
}


void MQTTClient::mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  err_t err;
  if(status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n");

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
    // err = mqtt_subscribe(client, IO_USERNAME "/feeds/" IO_FEED_NAME, 1, mqtt_sub_request_cb, arg);
    err = mqtt_subscribe(client, MQTT_TOPIC, 1, mqtt_sub_request_cb, arg);

    if(err != ERR_OK) {
      printf("mqtt_subscribe return: %d\n", err);
    } else {
        printf("mqtt_subscribe ok\n");
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);
  }
}




// DNS callback function that will be called when the DNS resolution is complete
void MQTTClient::dns_callback(const char *name, const ip4_addr_t *ipaddr, void *callback_arg) {
	// Copy the resolved IP address
    memcpy(&broker_ip, ipaddr, sizeof(broker_ip));

    // Give the semaphore to signal DNS resolution is complete
    xSemaphoreGiveFromISR(xSemaphoreDNSReady, NULL);
}

bool MQTTClient::isConnected(){
    if (client == NULL) {
        printf("Client is NULL\n");
        return false;
    }
    return mqtt_client_is_connected(client);
}


bool MQTTClient::isConnected(mqtt_client_t *client) {
  return mqtt_client_is_connected(client);
}

bool MQTTClient::tryConnect(mqtt_client_t *client) {
    xSemaphoreDNSReady = xSemaphoreCreateBinary();
    if (xSemaphoreDNSReady == NULL) {
        printf("Failed to create semaphore\n");
        return false;
    }

    if(client == NULL) {
        printf("Invalid client\n");
        return false;
    }

    // Start DNS request
    err_t dns_err = dns_gethostbyname(MQTT_BROKER_HOST, &broker_ip, dns_callback, client);

    if (dns_err == ERR_OK) {
        // DNS completed immediately
        xSemaphoreGive(xSemaphoreDNSReady);
    } else if (dns_err != ERR_INPROGRESS) {
        printf("DNS resolution failed: %d\n", dns_err);
        return false;
    }

    // Wait for DNS resolution to complete
    if (xSemaphoreTake(xSemaphoreDNSReady, pdMS_TO_TICKS(10000)) != pdTRUE) {
        printf("DNS resolution timed out\n");
        return false;
    }

    printf("Resolved IP address: %s\n", ipaddr_ntoa(&broker_ip));

    // Now call mqtt_client_connect
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = id.c_str();
    ci.client_user = IO_USERNAME;
    ci.client_pass = IO_KEY;

    err_t err = mqtt_client_connect(client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_cb, 0, &ci);
    if(err != ERR_OK) {
        printf("mqtt_connect return %d\n", err);
        return false;
    }

    return true;
}

void MQTTClient::sendMessages() {
    if (mqttSendQueue != NULL) {
        char* message;
        bool ready = false;
        // printf("messages in queue: %d\n", uxQueueMessagesWaiting(mqttSendQueue));
        while (mqtt_ready_to_send && inflight_messages < 1000) {
            if (xQueueReceive(mqttSendQueue, &message, 0) == pdTRUE) {
                ready = true;
                // printf("Sending message\n");
                publish_message(client, NULL, message);
                // free(message); // Free the allocated memory after sending
                // vTaskDelay(100); // Add a small delay to avoid flooding the queue
            } else {
                ready = true; // No messages to send so we are ready
                // printf("No messages to send\n");
                break; // Exit the loop if no messages are available
            }
        }
        if (!ready){ //Retry logic
            ticks_not_ready++;
            if (ticks_not_ready > 5) {
                mqtt_ready_to_send = true;
                ticks_not_ready = 0;
            }
        }
        // printf("Finished sending messages\n");
    }
}

void MQTTClient::addMessageToSend(std::string message){
    // printf("Adding message to send queue: %s\n", message.c_str());
    char* messagePtr = strdup(message.c_str());
    // const char* messagePtr = message.c_str();
    if (xQueueSend(mqttSendQueue, &messagePtr, 2) != pdTRUE){
        // printf("Failed to send message to queue\n");
        free(messagePtr); // Free the allocated memory if sending fails
    }
    // printf("Added message to send queue: %d\n", uxQueueMessagesWaiting(mqttSendQueue));
}

void MQTTClient::getIncomingMessages(std::vector<std::string> &messages) {
    if (mqttIncomingQueue != NULL) {
        std::string* messagePtr;
        while (xQueueReceive(mqttIncomingQueue, &messagePtr, 0) == pdTRUE) {
            if (messagePtr != nullptr) {
                // printf("Received message: %s\n", messagePtr->c_str());
                messages.push_back(*messagePtr);
                delete messagePtr; // Clean up
            }
        }
    }
}