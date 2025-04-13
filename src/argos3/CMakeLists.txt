add_library(argos3 STATIC
        core/utility/math/angles.cpp
        core/utility/math/angles.h
        core/utility/math/general.h
        core/utility/math/vector2.h
        core/utility/math/vector2.cpp
        core/config.h)

target_include_directories(argos3 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/core/utility/math # Add this line
)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/core/config.h.in
  ${CMAKE_CURRENT_BINARY_DIR}/core/config.h
  @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/core/config.h DESTINATION include/argos3/core)

target_include_directories(argos3 PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/core)

message(STATUS "installed config.h to ${CMAKE_CURRENT_BINARY_DIR}/core/config.h")