add_executable(${NAME}
        main.cpp
        WifiHelper.cpp
        # mqtt/MQTTEcho.c
        # mqtt/TestTrans.cpp
        mqtt/Agent.cpp
        mqtt/TCPTransport.cpp
        mqtt/TLSTransBlock.cpp
        mqtt/Transport.cpp
        mqtt/radio.cpp
        mqtt/MQTTCLient.cpp
        )

# add_library(paho_embedded_mqtt STATIC
#     # lib/paho.mqtt.embedded-c/MQTTClient-C/src/MQTTClient.c
#     ../lib/paho.mqtt.embedded-c/MQTTClient-C/src/FreeRTOS/MQTTFreeRTOS.c  #FreeRTOS
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTConnectClient.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTConnect.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTDeserializePublish.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTFormat.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTPacket.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTSerializePublish.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTSubscribeClient.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTSubscribe.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTUnsubscribeClient.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTUnsubscribe.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTDisconnect.c
#     # lib/paho.mqtt.embedded-c/MQTTPacket/src/MQTTTimer.c  # use if you don't override this
# )

# target_include_directories(paho_embedded_mqtt PUBLIC
#     lib/paho.mqtt.embedded-c/MQTTClient-C
#     lib/paho.mqtt.embedded-c/MQTTPacket/src
#     # ${CMAKE_SOURCE_DIR}/lib/FreeRTOS-Kernel/include    # Add this linep
# )

target_sources(${NAME} PRIVATE ${LWIP_CORE_SRC} ${LWIP_MQTT_SRC})
# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
    pico_stdlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    freertos_config #FREERTOS_PORT
    LWIP_PORT
    pico_cyw43_arch_lwip_sys_freertos
    # paho_embedded_mqtt
     coreHTTP
    #  urilib
    #  json_maker
    #  tiny_json
     wolfssl
    #  picow_wsclient
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
     )
     
# if (NOT DEFINED ENV{WIFI_SSID})
# 	message(FATAL_ERROR "WIFI_SSID Environment variable not defined")
# endif()
# if (NOT DEFINED ENV{WIFI_PASSWORD})
# 	message(FATAL_ERROR "WIFI_PASSWORD Environment variable not defined")
# endif()


# target_compile_definitions(${NAME} PRIVATE
#     WIFI_SSID=\"$ENV{WIFI_SSID}\"
#     WIFI_PASSWORD=\"$ENV{WIFI_PASSWORD}\"
#     CYW43_HOST_NAME="DrJonEA"
#     IPGEOLOCATION=\"$ENV{IPGEOLOCATION}\"
# )

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
target_compile_definitions(${NAME} PRIVATE
    PICO_DEFAULT_UART_RX_PIN=17
    PICO_DEFAULT_UART_TX_PIN=16
)