cmake_minimum_required(VERSION 3.5)
project(teb_local_planner)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(costmap_converter REQUIRED)
find_package(costmap_converter_msgs REQUIRED)
find_library(NLOPT_LIBRARY nlopt REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common QUIET)
#find_package(interactive_markers REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(teb_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

message(STATUS "System: ${CMAKE_SYSTEM}")
## System dependencies are found with CMake's conventions
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
message(STATUS "${CMAKE_MODULE_PATH}")
find_path(SUITESPARSE_INCLUDE_DIRS cholmod.h PATH_SUFFIXES suitesparse REQUIRED)
find_library(CHOLMOD_LIBRARY cholmod REQUIRED)
find_library(CXSPARSE_LIBRARY cxsparse REQUIRED)
set(SUITESPARSE_LIBRARIES ${CHOLMOD_LIBRARY} ${CXSPARSE_LIBRARY})
find_package(G2O REQUIRED)
# Boost.System is header-only; only Boost.Thread needs a library.
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED NO_MODULE)

set(EXTERNAL_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${Boost_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${EXTERNAL_INCLUDE_DIRS}
)

## Build the teb_local_planner library

add_library(teb_local_planner SHARED
   src/timed_elastic_band.cpp
   src/optimal_planner.cpp
   src/obstacles.cpp
   src/visualization.cpp
   src/recovery_behaviors.cpp
   src/teb_config.cpp
   src/homotopy_class_planner.cpp
   src/teb_local_planner_ros.cpp
   src/graph_search.cpp
)

target_include_directories(teb_local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EXTERNAL_INCLUDE_DIRS}
)

set(ament_dependencies
    costmap_converter
    costmap_converter_msgs
    nav2_core
    nav2_costmap_2d
    nav2_util
    geometry_msgs
    nav_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    std_msgs
    pluginlib
    teb_msgs
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
    builtin_interfaces
)
if(nav2_ros_common_FOUND)
  list(APPEND ament_dependencies nav2_ros_common)
endif()
if(COMMAND ament_target_dependencies)
  ament_target_dependencies(teb_local_planner
     ${ament_dependencies}
  )
else()
  foreach(dependency IN ITEMS ${ament_dependencies})
    if(dependency STREQUAL "Boost")
      target_link_libraries(teb_local_planner  Boost::headers)
    elseif(dependency STREQUAL "OpenCV")
      target_include_directories(teb_local_planner PUBLIC ${OpenCV_INCLUDE_DIRS})
      target_link_libraries(teb_local_planner  ${OpenCV_LIBRARIES})
    elseif(NOT DEFINED ${dependency}_TARGETS OR "${${dependency}_TARGETS}" STREQUAL "")
      message(FATAL_ERROR "${dependency} has no exported targets for teb_local_planner")
    else()
      target_link_libraries(teb_local_planner  ${${dependency}_TARGETS})
    endif()
  endforeach()
endif()
target_link_libraries(teb_local_planner
   ${EXTERNAL_LIBS}
   ${NLOPT_LIBRARY}
   Eigen3::Eigen
)

target_compile_definitions(teb_local_planner PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS teb_local_planner
   EXPORT teb_local_planner_targets
   DESTINATION lib
)
ament_export_targets(teb_local_planner_targets HAS_LIBRARY_TARGET)

## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION include/
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  teb_local_planner_plugin.xml
  DESTINATION share
  )

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(teb_local_planner)
ament_export_dependencies(${ament_dependencies})
pluginlib_export_plugin_description_file(nav2_core teb_local_planner_plugin.xml)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  #ament_add_gtest(homotopy_class_planner_test
  #    test/homotopy_class_planner_test.cpp
  #)
  #target_link_libraries(homotopy_class_planner_test
  #    teb_local_planner
  #)
endif()

ament_package(CONFIG_EXTRAS cmake_modules/teb_dependencies.cmake)
