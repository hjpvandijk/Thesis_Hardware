
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/config_yaml_data.h
    COMMAND xxd -i ${CMAKE_CURRENT_SOURCE_DIR}/agent_implementation/configs/hardware_config.yaml > ${CMAKE_CURRENT_BINARY_DIR}/config_yaml_data.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/agent_implementation/configs/hardware_config.yaml
    COMMENT "Generating config_yaml_data.h"
)

add_executable(${NAME}
        main.cpp
        pins.h
        WifiHelper.cpp
        AgentExecutor.cpp
        # mqtt/MQTTEcho.c
        # mqtt/TestTrans.cpp
        TaskAgent.cpp
        mqtt/TCPTransport.cpp
        # mqtt/TLSTransBlock.cpp
        mqtt/Transport.cpp
        mqtt/radio.cpp
        mqtt/MQTTCLient.cpp
        uart/UARTHandler.cpp
        motion/MotorController.cpp
        motion/DifferentialDrive.cpp
        # motion/MotorEncoder.cpp
        # motion/PID.cpp
        sensing/HC_SR04.cpp
        sensing/DistanceSensorHandler.cpp
        agent_implementation/agent.cpp
        agent_implementation/agent_control/battery/BatteryManager.cpp
        agent_implementation/agent_control/battery/MicroControllerBatterManager.cpp
        agent_implementation/agent_control/battery/MotionSystemBatteryManager.cpp
        agent_implementation/agent_control/communication/TimeSynchronizer.cpp
        agent_implementation/agent_control/path_planning/SimplePathPlanner.cpp
        agent_implementation/agent_control/path_planning/PathFollower.cpp
        agent_implementation/agent_control/path_planning/FrontierEvaluator.cpp
        agent_implementation/agent_control/path_planning/ForceVectorCalculator.cpp
        agent_implementation/agent_control/path_planning/RandomWalk.cpp
        agent_implementation/utils/Algorithms.cpp
        agent_implementation/utils/Quadtree.h
        agent_implementation/utils/Box.h
        agent_implementation/utils/CustomComparator.h
        agent_implementation/utils/coordinate.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/config_yaml_data.h
        )

# In src/CMakeLists.txt
target_compile_definitions(${NAME} PRIVATE
    PICO_COPY_TO_RAM=0            # Don't copy code to RAM by default
)

# For constant data that doesn't change
# # pico_set_binary_type(${NAME} copy_to_ram)
# pico_set_linker_script(${NAME} ${CMAKE_CURRENT_LIST_DIR}/custom_linker_script.ld)

# Memory optimization flags
set(MY_TARGET_MEMORY_OPTIMIZATION_FLAGS
    # -fno-exceptions                # Disable C++ exceptions to save space
    -ffunction-sections            # Place each function in its own section
    -fdata-sections                # Place each data item in its own section
    -finline-small-functions       # Inline small functions to reduce call overhead
    -fno-unwind-tables             # Disable generation of unwind tables
)

# Apply to all targets
target_compile_options(${NAME} PRIVATE ${MY_TARGET_MEMORY_OPTIMIZATION_FLAGS})

# target_link_options(${NAME} PRIVATE
#     -Wl,--gc-sections              # Remove unused sections
#     -Wl,--print-memory-usage  # Shows memory usage after linking
# )

# Create map file for memory analysis
set_target_properties(${NAME} PROPERTIES LINK_FLAGS "-Wl,-Map=${CMAKE_BINARY_DIR}/${NAME}.map")

target_sources(${NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/config_yaml_data.h) # Change is here.
target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}) # Add this line!
# add_subdirectory(agent_implementation)
add_subdirectory(argos3)


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


# Link directories (if you have specific libraries to link against)
include(FetchContent)

FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG master# Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
set(YAML_CPP_BUILD_TOOLS OFF)
FetchContent_MakeAvailable(yaml-cpp)

target_sources(${NAME} PRIVATE ${LWIP_CORE_SRC} ${LWIP_MQTT_SRC})
# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
    yaml-cpp::yaml-cpp
    argos3
    pico_stdlib
    hardware_pwm
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
    freertos_config #FREERTOS_PORT
    LWIP_PORT
    pico_cyw43_arch_lwip_sys_freertos
    # paho_embedded_mqtt
    #  coreHTTP
    #  urilib
    #  json_maker
    #  tiny_json
    #  wolfssl
    #  picow_wsclient

	)

	
# target_include_directories(${NAME} PRIVATE
#     ${CMAKE_CURRENT_LIST_DIR}
#     ${CMAKE_CURRENT_LIST_DIR}/Thesis_ARGoS/implementation_and_examples/agent_implementation
#     ${CMAKE_CURRENT_LIST_DIR}/argos3
#     ${CMAKE_CURRENT_LIST_DIR}/mqtt
#     ${CMAKE_CURRENT_LIST_DIR}/motion
#     ${CMAKE_CURRENT_LIST_DIR}/sensing

#      )


# message(STATUS "CMAKE_CURRENT_LIST_DIR: ${CMAKE_CURRENT_LIST_DIR}")
# add_dependencies(${NAME} argos3 agent_implementation)
     
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
    LWIP_TUNE
)