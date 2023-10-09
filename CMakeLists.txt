# Copyright (c) 2020-2021 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.5)
project(psen_scan_v2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-std=c++17)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)
add_compile_options(-Werror)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost 1.74)
if(NOT Boost_FOUND)
  find_package(Boost REQUIRED COMPONENTS system filesystem)
endif()
find_package(fmt REQUIRED)

################
## Clang tidy ##
################
# alternative to ament_cmake_clang_tidy which is very time consuming
# to run: catkin_make -DAMENT_ENABLE_CLANG_TIDY=true
# build and install folders have to be deleted before run
if(AMENT_ENABLE_CLANG_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
    )
  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found.")
  else()
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXE}")
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}") # Todo: -header-filter=${PROJECT_NAME}/.*
  endif()
endif()

######################
## Build Standalone ##
######################

set(${PROJECT_NAME}_standalone_sources
  standalone/src/scanner_v2.cpp
  standalone/src/laserscan.cpp
  standalone/src/data_conversion_layer/monitoring_frame_msg.cpp
  standalone/src/data_conversion_layer/start_request.cpp
  standalone/src/data_conversion_layer/start_request_serialization.cpp
  standalone/src/data_conversion_layer/stop_request_serialization.cpp
  standalone/src/data_conversion_layer/monitoring_frame_deserialization.cpp
  standalone/src/data_conversion_layer/diagnostics.cpp
  standalone/src/data_conversion_layer/scanner_reply_serialization_deserialization.cpp
)

add_library(${PROJECT_NAME}_standalone
  ${${PROJECT_NAME}_standalone_sources}
)
target_compile_definitions(${PROJECT_NAME}_standalone
  PUBLIC
  _ROS_BUILD_
)
target_include_directories(${PROJECT_NAME}_standalone
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/standalone/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_standalone
  Boost
  fmt
  rcutils
)

###########
## Build ##
###########

# ignore BOOST deprecated headers
add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")
add_definitions("-DBOOST_BIND_GLOBAL_PLACEHOLDERS")

set(${PROJECT_NAME}_node_sources
  src/psen_scan_driver.cpp
)

add_executable(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_node_sources}
)
target_include_directories(${PROJECT_NAME}_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_standalone
  fmt::fmt
)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
)

#############
## Install ##
#############

install(DIRECTORY include/ standalone/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_standalone
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
