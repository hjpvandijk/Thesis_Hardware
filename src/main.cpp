/***
 * main.cpp - HTTP Get over socket
 * Jon Durrant
 * 4-Oct-2022
 *
 *
 */

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include <stdio.h>

#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
// Undefine the conflicting macros
#undef write
#undef read
#undef bind

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
// #include "wolfssl/ssl.h"

#include "WifiHelper.h"
// #include "Request.h"
#include "config.h"
// #include "mqtt/TestTrans.h"
#include "mqtt/MQTTClient.h"
#include "mqtt/radio.h"
#include "uart/UARTHandler.h"
#include <vector>
#include <string>
#include "pico/unique_id.h"
#include "motion/MotorController.h"
#include "pins.h"
#include "sensing/DistanceSensorHandler.h"
#include "agent_implementation/agent.h"
#include "AgentExecutor.h"
#include "lwip/stats.h"




//Check these definitions where added from the makefile
#ifndef WIFI_SSID
#error "WIFI_SSID not defined"
#endif
#ifndef WIFI_PASSWORD
#error "WIFI_PASSWORD not defined"
#endif

#define TASK_PRIORITY     ( tskIDLE_PRIORITY + 1UL )
#define BUF_LEN					2048

pico_unique_board_id_t id;

void runTimeStats(){
  TaskStatus_t         * pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  unsigned long        ulTotalRunTime;


  /* Take a snapshot of the number of tasks in case it changes while this
  function is executing. */
  uxArraySize = uxTaskGetNumberOfTasks();
  printf("Number of tasks %d\n", uxArraySize);

  /* Allocate a TaskStatus_t structure for each task.  An array could be
  allocated statically at compile time. */
  pxTaskStatusArray = (TaskStatus_t*) pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

  if (pxTaskStatusArray != NULL){
    /* Generate raw status information about each task. */
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,
                                       uxArraySize,
                                       &ulTotalRunTime);



    /* For each populated position in the pxTaskStatusArray array,
    format the raw data as human readable ASCII data. */
    for (x = 0; x < uxArraySize; x++){
      printf("Task: %d \t cPri:%d \t bPri:%d \t hw:%d \t%s\n",
             pxTaskStatusArray[x].xTaskNumber,
             pxTaskStatusArray[x].uxCurrentPriority,
             pxTaskStatusArray[x].uxBasePriority,
             pxTaskStatusArray[x].usStackHighWaterMark,
             pxTaskStatusArray[x].pcTaskName
      );
    }


    /* The array is no longer needed, free the memory it consumes. */
    vPortFree(pxTaskStatusArray);
  } else{
    printf("Failed to allocate space for stats\n");
  }

  HeapStats_t heapStats;
  vPortGetHeapStats(&heapStats);
  printf("HEAP avl: %d, blocks %d, alloc: %d, free: %d\n",
         heapStats.xAvailableHeapSpaceInBytes,
         heapStats.xNumberOfFreeBlocks,
         heapStats.xNumberOfSuccessfulAllocations,
         heapStats.xNumberOfSuccessfulFrees
  );

}

void runTimeStatsLWIP() {
  stats_display();
}




void debugCB(const int logLevel, const char *const logMessage){
	printf("WOLFSSL DEBUG(%d): %s\n", logLevel, logMessage);
}


// void main_task(void* params){

// 	printf("Main task started\n");

// 	// wolfSSL_Init();
// 	// wolfSSL_SetLoggingCb( debugCB);
// 	//wolfSSL_Debugging_ON();


// 	if (WifiHelper::init()){
// 	printf("Wifi Controller Initialised\n");
// 	} else {
// 	printf("Failed to initialise controller\n");
// 	return;
// 	}




// 	printf("Connecting to WiFi... %s \n", WIFI_SSID);

// 	if (WifiHelper::join(WIFI_SSID, WIFI_PASSWORD)){
// 	printf("Connect to Wifi\n");
// 	}
// 	else {
// 	printf("Failed to connect to Wifi \n");
// 	}


// 	//Print MAC Address
// 	char macStr[20];
// 	WifiHelper::getMACAddressStr(macStr);
// 	printf("MAC ADDRESS: %s\n", macStr);

// 	//Print IP Address
// 	char ipStr[20];
// 	WifiHelper::getIPAddressStr(ipStr);
// 	printf("IP ADDRESS: %s\n", ipStr);


// 	//Call IPGeo Web Service
// 	char userBuf[BUF_LEN];
// 	// Request req((char *)userBuf, BUF_LEN);
// 	bool res;
// 	//char url[] = "https://vmu22a.local.jondurrant.com:5443/time";
// 	// char url[] = "https://api.ipgeolocation.io/ipgeo";
// 	//char url[] = "http://vmu22a.local.jondurrant.com:5000/args";
// 	//char url[] = "https://vmu22a.local.jondurrant.com:5443/args";

// 	// std::map<std::string, std::string> query;

// 	// query["apiKey"]=IPGEOLOCATION;
// 	runTimeStats();
// 	// res = req.get(url, &query);
// 	// if ( res ){
// 	// 	res = (req.getStatusCode() == 200);
// 	// }
// 	// if (res){
// 	// 	printf("IPGeo: %.*s\n", req.getPayloadLen(), req.getPayload());
// 	// } else {
// 	// 	printf("IPGeo failed %d\n", req.getStatusCode());
// 	// }

// 	runTimeStats();



// 	while (true){

// 	runTimeStats();

// 	vTaskDelay(3000);


// 	if (!WifiHelper::isJoined()){
// 	  printf("AP Link is down\n");

// 	  if (WifiHelper::join(WIFI_SSID, WIFI_PASSWORD)){
// 		printf("Connect to Wifi\n");
// 	  } else {
// 		printf("Failed to connect to Wifi \n");
// 	  }
// 	} else {
// 		printf("AP Link is up\n");
// 	}

// 	}

// }

std::string getPicoUniqueID() {
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);

  char buffer[PICO_UNIQUE_BOARD_ID_SIZE_BYTES * 2 + 1];
  for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; ++i) {
    snprintf(&buffer[i * 2], 3, "%02X", id.id[i]);
  }

  return std::string(buffer);
}

size_t getTotalHeapSize() {
    extern char __StackLimit; // Defined by the linker script
    extern char __bss_end__; // Defined by the linker script

    size_t totalHeapSize = &__StackLimit - &__bss_end__;
    return totalHeapSize;
}

void main_task(void* params){

  std::string uniqueID = getPicoUniqueID();

  //Assign random agent id

  printf("Main task started\n");

  if (WifiHelper::init()){
    printf("Wifi Controller Initialised\n");
  } else {
    printf("Failed to initialise controller\n");
    return;
  }


  printf("Connecting to WiFi... %s \n", WIFI_SSID);

  if (WifiHelper::join(WIFI_SSID, WIFI_PASSWORD)){
    printf("Connect to Wifi\n");
  }
  else {
    printf("Failed to connect to Wifi \n");
  }


  //Print MAC Address
  char macStr[20];
  WifiHelper::getMACAddressStr(macStr);
  printf("MAC ADDRESS: %s\n", macStr);

  //Print IP Address
  char ipStr[20];
  WifiHelper::getIPAddressStr(ipStr);
  printf("IP ADDRESS: %s\n", ipStr);


  // TestTrans testTrans;
  // testTrans.start("test", TASK_PRIORITY);
  vTaskDelay(4000);

  printf("Connecting to MQTT broker...\n");
  MQTTClient mqttClient(uniqueID);
  mqttClient.start("mqtt", TASK_PRIORITY+6);

  printf("Making radio\n");
  Radio radio(&mqttClient);

  UARTHandler uartHandler;
  uartHandler.start("uart", TASK_PRIORITY+1);

  MotorController motorController;
  motorController.setLeftMotorSpeed(220);
  motorController.setRightMotorSpeed(220);

  bool motorsRunning = true;

  DistanceSensorHandler distanceSensorHandler;
  distanceSensorHandler.start("distance", TASK_PRIORITY + 10);
  printf("creating agent\n");

  AgentExecutor agentExecutor(uniqueID);
  agentExecutor.agent.setWifi(radio);

  //Set agent hc_sr04 sensors to distancesensorhandler sensors
  // for (int i = 0; i < 4; i++) {
  //   agent.distance_sensors[i] = distanceSensorHandler.sensors[i];
  // }
  agentExecutor.agent.setDistanceSensorHandler(&distanceSensorHandler);

  if (!agentExecutor.start("agent", TASK_PRIORITY + 1)) {
      printf("Failed to start agent task!\n");
  }  
  printf("starting loop\n");
  int i = 0;
  // double x = -4;
  // double y = -4;
  // double x_step = 0;
  // double y_step = 0.1;
  while (true){
    // printf("Main task running\n");
    // agentExecutor.agent.setPosition(uartHandler.getPosition().x, uartHandler.getPosition().y);
    agentExecutor.agent.setHeading(uartHandler.getHeading());
    // printf("heading: %f\n", ToDegrees(uartHandler.getHeading()));
    // x += x_step; 
    // y += y_step;
    // printf("Agent position: %f, %f\n", x, y);
    // printf("%d, %d\n", x == -4, y == 4);
    // if ((abs(x+4) < 1e-6) && (abs(y-4) < 1e-6)){ // x==-4 && y==4
    //   x_step = 0.1;
    //   y_step = 0;
    // } else if (abs(x-4) < 1e-6 && abs(y-4) < 1e-6){ // x==4 && y==44
    //   x_step = 0;
    //   y_step = -0.1;
    // } else if (abs(x-4) < 1e-6 && abs(y+4) < 1e-6){ // x==4 && y==-4
    //   x_step = -0.1;
    //   y_step = 0;
    // } else if (abs(x+4) < 1e-6 && abs(y+4) < 1e-6){ // x==-4 && y==-4
    //   x_step = 0;
    //   y_step = 0.1;
    // }
    // if (i%500==0){
    // runTimeStats();
    // size_t freeHeapSize = xPortGetFreeHeapSize();
    // printf("Free heap size: %zu\n", freeHeapSize);
    // size_t totalHeapSize = getTotalHeapSize();
    // printf("Total heap size: %zu\n", totalHeapSize);

    // printf("LWIP:\n");
    // runTimeStatsLWIP();
    
    // }

    // std::string message = "Hello World " + std::to_string(i++);
    // radio.send_message(message, "agent1");

    // std::string message1 = "[7E6E2E794F85C86F]C:0.000000;0.000000|0.314405;0.020960 " + std::to_string(i);
    // std::string message2 = "[7E6E2E794F85C86F]V:1.000000;0.000000 " + std::to_string(i++);
    // radio.broadcast_message(message1);
    // radio.broadcast_message(message2);


// for (int i = 0; i < 4; i++) {
//       // if (i != 1) continue;
//       float distance = distanceSensorHandler.getDistance(i);
//       // float distanceAgent = agent.distance_sensors.at(i)->getDistance();
//       // agent.setLastRangeReadings(i, distance);
//       printf("%d: %f\t", i, distance);
//       // printf("%d-A: %f\t", i, distanceAgent);
//     }
//     printf("\n");

    // printf("Main task running\n");

    

    // // runTimeStats();
    // std::string bMessage = "Hello World " + std::to_string(i++);
    // std::string bMessagePrependedWithId = "[" + uniqueID + "]" + bMessage;
    // radio.broadcast_message(bMessagePrependedWithId);
    // std::string sMessage = "Your World " + std::to_string(i++);
    // std::string sMessagePrependedWithId = "[" + uniqueID + "]" + sMessage;
    // radio.send_message(sMessagePrependedWithId, "agent1");

    // // mqttClient.addMessageToSend("Hello World " + std::to_string(i++));
    // std::vector<std::string> messages;
    // radio.receive_messages(messages, 0.0);
    // for (const auto& message : messages) {
    //   printf("Received message in vector: %s\n", message.c_str());
    // }

    // printf("Main task running\n");

    


    // if (motorsRunning){
    //   motorController.setLeftMotorSpeed(220);
    //   motorController.setRightMotorSpeed(-220);
    //   motorsRunning = false;
    //   printf("Turning right\n");
    // } else {
    //   motorController.setLeftMotorSpeed(-220);
    //   motorController.setRightMotorSpeed(220);
    //   motorsRunning = true;
    //   printf("Turning left\n");
    // }

    // if (!WifiHelper::isJoined()){
    //   printf("AP Link is down\n");

    //   if (WifiHelper::join(WIFI_SSID, WIFI_PASSWORD)){
    //     printf("Connect to Wifi\n");
    //   } else {
    //     printf("Failed to connect to Wifi \n");
    //   }
    // }
    vTaskDelay(1000/8);


  }

}


void vLaunch(void) {
  TaskHandle_t task;

  xTaskCreate(main_task, "MainThread", 3000, NULL, TASK_PRIORITY, &task);

  /* Start the tasks and timer running. */
  vTaskStartScheduler();
}


int main(void) {
  stdio_init_all();
  sleep_ms(2000);
  printf("GO\n");


  /* Configure the hardware ready to run the demo. */
  const char* rtos_name;
  rtos_name = "FreeRTOS";
  printf("Starting %s on core 0:\n", rtos_name);
  vLaunch();

  return 0;
}
