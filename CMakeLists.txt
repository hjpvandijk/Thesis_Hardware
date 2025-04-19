# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.12)
set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")


set(NAME BICLARE_HARDWARE) 
set(PICO_BOARD pico2_w)
set(PICO_PLATFORM rp2350)

# include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")
include(pico_sdk_import.cmake)


project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

# Set optimization flagsp
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Release build optimizations
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -flto")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -flto")

#flto may be too aggressive for some libraries
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -Os -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Os -DNDEBUG")

# # Debug build options (lower optimization, more debugging info)
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og -g3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og -g3")


# Linker optimization
add_link_options(
    -Wl,--print-memory-usage      # Print memory usage summary during build
    -Wl,-Map=output.map  # Create a map file for memory analysis
)

# Print the build type
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# # Print the build type
# message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# Initialize the SDK
pico_sdk_init()


include(lwip_import.cmake)

# SET(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/RPIPicoWWSClient-RTOS/lib/FreeRTOS-Kernel")
# SET(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel")
SET(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel")


# SET(FREERTOS_KERNEL_PATH "$ENV{PICO_FREERTOS}" CACHE STRING "Course Common Lib")
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel") # CACHE STRING "Local Config")
include(${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel/freertos_config.cmake)
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
include(FreeRTOS_Kernel_import.cmake)

# set(COREHTTP_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/RPIPicoWWSClient-RTOS/lib/coreHTTP")# CACHE STRING "Course Common Lib")
# set(COREHTTP_PORT "${CMAKE_CURRENT_LIST_DIR}/port/coreHTTP/") # CACHE STRING "Course Common Lib")
# include(coreHTTP.cmake)

# SET(URILIB_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/RPIPicoWWSClient-RTOS/lib/uri-library") # CACHE STRING "Common Lib")
# include(urilib.cmake)

# SET(JSON_MAKER_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/RPIPicoWWSClient-RTOS/lib/json-maker") # CACHE STRING "Common Lib")
# include(json-maker.cmake)

# SET(TINY_JSON_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/RPIPicoWWSClient-RTOS/lib//tiny-json") # CACHE STRING "Common Lib")
# include(tiny-json.cmake)

# set(WOLFSSL_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/RPIPicoWWSClient-RTOS/lib/wolfssl") # CACHE STRING "Course Common Lib")
# set(WOLFSSL_DIR "${CMAKE_CURRENT_LIST_DIR}/lib/wolfssl") # CACHE STRING "Course Common Lib")
# include("wolfssl_import.cmake")

# set(PICOW_WSCLIENT_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/RPIPicoWWSClient-RTOS")# CACHE STRING "Course Common Lib")
# include("PicoW_WSClient.cmake")




add_subdirectory(src)

#Set up files for the release packages
install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
