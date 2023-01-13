
cmake_minimum_required(VERSION 3.4)
project(flexiv_rdk_teleop-examples)

set(CMAKE_CXX_STANDARD 14)

message("OS: ${CMAKE_SYSTEM_NAME}")
message("Processor: ${CMAKE_SYSTEM_PROCESSOR}")

# Configure build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

# Minumum example list for all OS
set(EXAMPLE_LIST
  direct_pose_teleop
)

# Find flexiv_rdk_teleop INTERFACE library
find_package(flexiv_rdk_teleop REQUIRED)

# Build all selected examples
foreach(example ${EXAMPLE_LIST})
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} flexiv::flexiv_rdk_teleop)
endforeach()
