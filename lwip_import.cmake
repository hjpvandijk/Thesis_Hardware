add_library(LWIP_PORT INTERFACE)
target_include_directories(LWIP_PORT
    INTERFACE
       ${CMAKE_CURRENT_LIST_DIR}/port/lwip
    #   ${PICO_LWIP_PATH}/src/include/
    )

# message(STATUS "PICO_LWIP_PATH: ${PICO_LWIP_PATH}")
    
# set(LWIP_CONTRIB_DIR ${PICO_LWIP_CONTRIB_PATH})
# include(${PICO_LWIP_CONTRIB_PATH}/Filelists.cmake)
# add_subdirectory(${PICO_LWIP_PATH} lwip)

set(LWIP_MQTT_SRC
    ${PICO_LWIP_PATH}/src/apps/mqtt/mqtt.c
    # ${PICO_LWIP_PATH}/src/apps/mqtt/mqtt_client.c
)
