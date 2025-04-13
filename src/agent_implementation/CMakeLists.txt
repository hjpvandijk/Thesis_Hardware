# Include directories at the top-level include paths
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})  # Parent directory of cereal
# Define the library
add_library(agent_implementation STATIC
        feature_config.h
        agent.h
        agent.cpp
        configs
        utils/coordinate.h
        utils/coordinate.cpp
        utils/Quadtree.h
        utils/Box.h
        utils/CustomComparator.h
        utils/Algorithms.h
        utils/Algorithms.cpp
        # agent_control/communication/simulation/radio.h
        # agent_control/communication/simulation/radio.cpp
        agent_control/communication/TimeSynchronizer.h
        agent_control/communication/TimeSynchronizer.cpp

        # agent_control/sensing/simulation/distance_sensor/hc_sr04.h
        # agent_control/sensing/simulation/distance_sensor/hc_sr04.cpp
        # agent_control/motion/simulation/DifferentialDrive.h
        # agent_control/motion/simulation/DifferentialDrive.cpp
        # agent_control/path_planning/WallFollower.h
        # agent_control/path_planning/WallFollower.cpp
        agent_control/path_planning/SimplePathPlanner.h
        agent_control/path_planning/SimplePathPlanner.cpp
        agent_control/path_planning/PathFollower.h
        agent_control/path_planning/PathFollower.cpp
        agent_control/path_planning/FrontierEvaluator.h
        agent_control/path_planning/FrontierEvaluator.cpp
        agent_control/path_planning/ForceVectorCalculator.h
        agent_control/path_planning/ForceVectorCalculator.cpp
        agent_control/path_planning/RandomWalk.h
        agent_control/path_planning/RandomWalk.cpp
        agent_control/battery/BatteryManager.h
        agent_control/battery/BatteryManager.cpp
        agent_control/battery/MicroControllerBatteryManager.h
        agent_control/battery/MicroControllerBatterManager.cpp
        agent_control/battery/MotionSystemBatteryManager.h
        agent_control/battery/MotionSystemBatteryManager.cpp
)

# Link directories (if you have specific libraries to link against)
include(FetchContent)

FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG master# Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

target_include_directories(agent_implementation
    PUBLIC 
)

message(STATUS "CMakeLists.txt: agent_implementation include directories: ${CMAKE_SOURCE_DIR}/src")

target_link_libraries(agent_implementation PUBLIC 
        yaml-cpp::yaml-cpp
        argos3
        ) # The library or executable that require yaml-cpp library


# target_include_directories(agent_implementation PUBLIC
#     ${CMAKE_SOURCE_DIR}/src/argos3/core/utility/math
#     ${CMAKE_SOURCE_DIR}/src/argos3/core/utility/math/matrix
#     ${CMAKE_SOURCE_DIR}/src/argos3/core/utility/datatypes
#     ${CMAKE_SOURCE_DIR}/src/
# )
message(STATUS "CMakeLists.txt: agent_implementation include directories: ${CMAKE_SOURCE_DIR}/src/argos3")
