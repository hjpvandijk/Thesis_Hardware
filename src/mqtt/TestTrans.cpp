/*
 * TestTrans.cpp
 *
 *  Created on: 4 Sept 2022
 *      Author: jondurrant
 */

#include "TestTrans.h"
#include "WifiHelper.h"
#include "TCPTransport.h"
#include "TLSTransBlock.h"
#include "../config.h"
// #include "transport_interface.h"


// Define the MQTT connection parameters
#define MQTT_BROKER_HOST "io.adafruit.com"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "TestClient"
#define MQTT_TOPIC "test/topic"
#define MQTT_MESSAGE "Hello, MQTT"
ip_addr_t broker_ip;
SemaphoreHandle_t xSemaphoreDNSReady = NULL;


TestTrans::TestTrans() {
	// TODO Auto-generated constructor stub

}

TestTrans::~TestTrans() {
	// TODO Auto-generated destructor stub
}

// void TestTrans::run(){

// 	test();

// 	printf("RUN %d TESTS, SUCESSFUL %d\n", xTests, xSuccessful);

// 	while (true) { // Loop forever
//         // test();
// 		vTaskDelay(2000);
// 	}

// }

void TestTrans::run() {
    mqtt_client_t *client = mqtt_client_new();
    if (client == NULL) {
        printf("Failed to create MQTT client\n");
        return;
    }

    // Initial connection
    tryConnect(client);

    while (true) {
        if (!isConnected(client)) {
            printf("Disconnected. Reconnecting...\n");
            tryConnect(client);
        }
        
        // Your logic here (e.g., publishing, handling messages, etc.)
        example_publish(client, NULL);
        vTaskDelay(2000);  // Wait for 2 seconds before checking again
    }
}



/***
* Get the static depth required in words
* @return - words
*/
configSTACK_DEPTH_TYPE TestTrans::getMaxStackSize(){
	return 10000;
}

void TestTrans::test(){

	// xTests++;
	// if (!testConnect()){
	// 	printf("CONNECTIONED FAILED\n");
	// } else {
	// 	xSuccessful++;
	// }

	xTests++;
	if (!testTrans()){
		printf("Trans FAILED\n");
	} else {
		xSuccessful++;
	}

	// xTests++;
	// if (!testTLS()){
	// 	printf("Trans FAILED\n");
	// } else {
	// 	xSuccessful++;
	// }


}


bool TestTrans::testConnect(){
	if (WifiHelper::isJoined()) {
		return true;
	}
	if (WifiHelper::join(WIFI_SSID, WIFI_PASSWORD)){
		if (WifiHelper::isJoined()) {
			return true;
		} else {
			printf("is Joined is false\n\r");
		}
	} else {
		printf("Connect failed\n\r");
	}

	return false;
}

// bool TestTrans::testTrans(){
// 	uint16_t targetPort = 80;
// 	char targetHost[]="drjonea.co.uk";
// 	char message[]="GET / HTTP/1.1\r\n"
// 					"Host: drjonea.co.uk\r\n"
// 					"Connection: close\r\n"
// 					"\r\n";
// 	char buf[1024];
// 	int32_t retVal;

// 	TCPTransport sockTrans;

// 	if (!sockTrans.transConnect(targetHost, targetPort)){
// 		printf("Socket Connect Failed\r\n");
// 		return false;
// 	}

// 	retVal = sockTrans.transSend( message, strlen(message));
// 	if (retVal != strlen(message)){
// 		printf("Socket Send failed\n\r");
// 		return false;
// 	}

// 	retVal = 1;

// 	while (retVal > 0){
// 		retVal = sockTrans.transRead( buf, sizeof(buf));
// 		if (retVal > 0){
// 			sockTrans.debugPrintBuffer("READ:", buf, retVal);
// 		}
// 	}

// 	sockTrans.transClose();



// 	return true;
// }

/* Called when publish is complete either with success or failure */
void TestTrans::mqtt_pub_request_cb(void *arg, err_t result)
{
  if(result != ERR_OK) {
    printf("Publish result: %d\n", result);
  }
}

void TestTrans::example_publish(mqtt_client_t *client, void *arg)
{
  const char *pub_payload= "PubSubHubLubJub";
  err_t err;
  u8_t qos = 2; /* 0 1 or 2, see MQTT specification */
  u8_t retain = 0; /* No don't retain such crappy payload... */
  printf("example_publish: Publishing to topic %s\n", "pub_topic");
  err = mqtt_publish(client, IO_USERNAME "/feeds/" IO_FEED_NAME, pub_payload, strlen(pub_payload), qos, retain, mqtt_pub_request_cb, arg);
  if(err != ERR_OK) {
    printf("Publish err: %d\n", err);
  }
  else {
    printf("Publish ok\n");
  }
}

// void TestTrans::listenForMessages(mqtt_client_t *client) {
//     if (client == NULL) {
//         printf("Invalid MQTT client\n");
//         return;
//     }

//     while (true) {
//         if (!isConnected(client)) {
//             printf("Disconnected. Reconnecting...\n");
//             tryConnect(client);
//         }

//         // Handle incoming messages
//         mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, NULL);

//         // Delay to avoid busy looping
//         vTaskDelay(1000);
//     }
// }




/* The idea is to demultiplex topic and create some reference to be used in data callbacks
   Example here uses a global variable, better would be to use a member in arg
   If RAM and CPU budget allows it, the easiest implementation might be to just take a copy of
   the topic string and use it in mqtt_incoming_data_cb
*/
static int inpub_id;
void TestTrans::mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  printf("Incoming publish at topic %s with total length %u\n", topic, (unsigned int)tot_len);

  /* Decode topic string into a user defined reference */
  if(strcmp(topic, IO_USERNAME "/feeds/" IO_FEED_NAME) == 0) {
    inpub_id = 0;
  } else if(topic[0] == 'A') {
    /* All topics starting with 'A' might be handled at the same way */
    inpub_id = 1;
  } else {
    /* For all other topics */
    inpub_id = 2;
  }
}

void TestTrans::mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  printf("Incoming publish payload with length %d, flags %u\n", len, (unsigned int)flags);

  if(flags & MQTT_DATA_FLAG_LAST) {
    /* Last fragment of payload received (or whole part if payload fits receive buffer
       See MQTT_VAR_HEADER_BUFFER_LEN)  */

    /* Call function or do action depending on reference, in this case inpub_id */
    if(inpub_id == 0) {
        printf("inpub_id == 0\n");
        //Add own null terminator
          char *data_copy = (char *)malloc(len + 1);
            if (data_copy != NULL) {
                memcpy(data_copy, data, len);
                data_copy[len] = '\0'; // Null-terminate the string
                printf("mqtt_incoming_data_cb: %s\n", data_copy);
                free(data_copy);
            } else {
                printf("Memory allocation failed\n");
            }
      /* Don't trust the publisher, check zero termination */
    //   if(data[len-1] == 0) {
    //     printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
    //   } else {
    //     printf("mqtt_incoming_data_cb: Ignoring payload, not zero terminated...\n");
    //   }
        printf("mqtt_incoming_data_cb: ");
        fwrite(data, 1, len, stdout);
        printf("\n");
        printf("mqtt_incoming_data_cb: %s\n", (const char *)data);
        printf("Payload bytes (%d): ", len);
        for (int i = 0; i < len; i++) {
            printf("%02X ", data[i]);
        }
        printf("\n");
    } else if(inpub_id == 1) {
      /* Call an 'A' function... */
    } else {
      printf("mqtt_incoming_data_cb: Ignoring payload...\n");
    }
  } else {
    /* Handle fragmented payload, store in buffer, write to file or whatever */
  }
}

void TestTrans::mqtt_sub_request_cb(void *arg, err_t result)
{
  /* Just print the result code here for simplicity,
     normal behaviour would be to take some action if subscribe fails like
     notifying user, retry subscribe or disconnect from server */
  printf("Subscribe result: %d\n", result);
}

void TestTrans::example_do_connect(mqtt_client_t *client)
{
  struct mqtt_connect_client_info_t ci;
  err_t err;

  /* Setup an empty client info structure */
  memset(&ci, 0, sizeof(ci));

  /* Minimal amount of information required is client identifier, so set it here */
  ci.client_id = "lwip_test";

  /* Initiate client and connect to server, if this fails immediately an error code is returned
     otherwise mqtt_connection_cb will be called with connection result after attempting
     to establish a connection with the server.
     For now MQTT version 3.1.1 is always used */

err_t dns_err = dns_gethostbyname(MQTT_BROKER_HOST, &broker_ip, dns_callback, client);

  /* Wait for DNS resolution to complete */
  while (dns_err == ERR_INPROGRESS) {
    vTaskDelay(100);
    dns_err = dns_gethostbyname(MQTT_BROKER_HOST, &broker_ip, dns_callback, client);
  }
    if (dns_err != ERR_OK) {
        printf("DNS resolution failed: %d\n", dns_err);
        return;
    }


err = mqtt_client_connect(client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_cb, 0, &ci);

  /* For now just print the result code if something goes wrong */
  if(err != ERR_OK) {
    printf("mqtt_connect return %d\n", err);
  }
}

void TestTrans::mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  err_t err;
  if(status == MQTT_CONNECT_ACCEPTED) {
    printf("mqtt_connection_cb: Successfully connected\n");

    /* Setup callback for incoming publish requests */
    mqtt_set_inpub_callback(client, mqtt_incoming_publish_cb, mqtt_incoming_data_cb, arg);

    /* Subscribe to a topic named "subtopic" with QoS level 1, call mqtt_sub_request_cb with result */
    err = mqtt_subscribe(client, IO_USERNAME "/feeds/" IO_FEED_NAME, 1, mqtt_sub_request_cb, arg);

    if(err != ERR_OK) {
      printf("mqtt_subscribe return: %d\n", err);
    } else {
        printf("mqtt_subscribe ok\n");
    }
  } else {
    printf("mqtt_connection_cb: Disconnected, reason: %d\n", status);

    /* Its more nice to be connected, so try to reconnect */
	example_do_connect(client);
  }
}




// DNS callback function that will be called when the DNS resolution is complete
void TestTrans::dns_callback(const char *name, const ip4_addr_t *ipaddr, void *callback_arg) {
	// Copy the resolved IP address
    memcpy(&broker_ip, ipaddr, sizeof(broker_ip));

    // Give the semaphore to signal DNS resolution is complete
    xSemaphoreGiveFromISR(xSemaphoreDNSReady, NULL);
}



// bool TestTrans::testTrans() {
// 	mqtt_client_t *client = mqtt_client_new();
//     xSemaphoreDNSReady = xSemaphoreCreateBinary();
//     if (xSemaphoreDNSReady == NULL) {
//         printf("Failed to create semaphore\n");
//         return false;
//     }
// 	if(client != NULL) {
// 		example_do_connect(client);
// 	} else {
// 		printf("Failed to create MQTT client\n");
// 	}

//     //Test publish
//     // Wait for DNS resolution to complete
//     if (xSemaphoreTake(xSemaphoreDNSReady, pdMS_TO_TICKS(10000)) == pdTRUE) {
//         printf("DNS resolution completed\n");
//     } else {
//         printf("DNS resolution timed out\n");
//         return false;
//     }
//     // Now you can use the resolved IP address (broker_ip) to connect to the MQTT broker
//     // Example: Print the resolved IP address
//     printf("Resolved IP address: %s\n", ipaddr_ntoa(&broker_ip));
//     // Example: Connect to the MQTT broker using the resolved IP address
//     if (client != NULL){
//         example_publish(client, NULL);
//     }   
//     else {
//         printf("Failed to create MQTT client\n");
//         return false;
//     }

//     // Clean up
//     mqtt_disconnect(client);
//     mqtt_client_free(client);

// 	return true;

// }

bool TestTrans::isConnected(mqtt_client_t *client) {
    return mqtt_client_is_connected(client);
}

bool TestTrans::tryConnect(mqtt_client_t *client) {
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
    ci.client_id = "lwip_test";
    ci.client_user = IO_USERNAME;
    ci.client_pass = IO_KEY;

    err_t err = mqtt_client_connect(client, &broker_ip, MQTT_BROKER_PORT, mqtt_connection_cb, 0, &ci);
    if(err != ERR_OK) {
        printf("mqtt_connect return %d\n", err);
        return false;
    }

    return true;
}


bool TestTrans::testTLS(){
	uint16_t targetPort = 443;
	char targetHost[]="drjonea.co.uk";
	char get[]="GET /test/ HTTP/1.1\r\n"
					"Host: drjonea.co.uk\r\n"
					"Connection: close\r\n"
					"\r\n";
	char buf[1024];
	int32_t retVal;
	int err;
	int count=0;
	int waitCount;

	TLSTransBlock sockTrans;

	printf("Testing TLS\n");

	if (!sockTrans.transConnect(targetHost, targetPort)){
		printf("Socket Connect Failed\r\n");
		return false;
	}

	printf("Test TLS Connected \n");


	retVal = 1;
	waitCount = 0;
	while (retVal >= 0) {
		retVal = sockTrans.transRead( buf, 1);

		if (retVal == 0){
			waitCount ++;
			vTaskDelay(300);
		} else {
			retVal = sockTrans.transRead( &buf[1], sizeof(buf)-1);

			if (retVal > 0){
				sockTrans.debugPrintBuffer("READ:", buf, retVal);
			}
		}

		if (waitCount > 10){
			break;
		}


	}

	printf("#######HTTP HEAD END#######\n");


	printf("#######HTTP GET START#######\n");
	retVal = sockTrans.transSend( get, strlen(get));
	if (retVal != strlen(get)){
		printf("Socket Send failed\n\r");
		return false;
	}

	retVal = 1;
	waitCount = 0;
	while (retVal >= 0) {
		retVal = sockTrans.transRead( buf, 1);

		if (retVal == 0){
			waitCount ++;
			vTaskDelay(300);
		} else {
			retVal = sockTrans.transRead( &buf[1], sizeof(buf)-1);

			if (retVal > 0){
				sockTrans.debugPrintBuffer("READ:", buf, retVal);
				count = count + retVal;
			}
		}

		if (waitCount > 10){
			break;
		}

		if (count  > 2048){
			printf("Truncating returned data\n");
			break;
		}


	}

	printf("#######HTTP GET END#######\n");


	sockTrans.transClose();



	return true;
}
