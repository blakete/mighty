cmake_minimum_required(VERSION 3.8)
project(mighty)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Default build type" FORCE)
  message(STATUS "mighty: defaulting CMAKE_BUILD_TYPE to Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynus_interfaces REQUIRED)
find_package(decomp_util REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# Only common + kdtree are used, but pcl_conversions re-finds PCL without
# components and exports every PCL library, so narrowing here would not help.
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# include directories
include_directories(${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Everything the planner (and the tests that compile planner sources) includes.
set(PLANNER_DEPS
  rclcpp dynus_interfaces decomp_ros_msgs Eigen3 pcl_conversions
  geometry_msgs nav_msgs sensor_msgs std_msgs visualization_msgs
  tf2 tf2_ros tf2_geometry_msgs)

# add executables
add_executable(mighty src/mighty/mighty_node.cpp src/mighty/mighty.cpp src/hgp/hgp_manager.cpp src/hgp/utils.cpp src/hgp/hgp_planner.cpp src/hgp/graph_search.cpp src/mighty/utils.cpp src/mighty/lbfgs_solver.cpp src/mighty/lbfgs_solver_utils.cpp src/mighty/frontier_detector.cpp src/mighty/frontier_manager.cpp)
target_link_libraries(mighty ${PCL_LIBRARIES})
ament_target_dependencies(mighty ${PLANNER_DEPS})

# nav_msgs/Odometry -> dynus_interfaces/State
add_executable(convert_odom_to_state src/mighty/convert_odom_to_state.cpp)
ament_target_dependencies(convert_odom_to_state rclcpp dynus_interfaces geometry_msgs nav_msgs)

# Mocap PoseStamped + TwistStamped -> dynus_interfaces/State
add_executable(convert_vicon_to_state src/mighty/convert_vicon_to_state.cpp)
ament_target_dependencies(convert_vicon_to_state rclcpp dynus_interfaces geometry_msgs message_filters)

# install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# install executables
install(TARGETS
  mighty
  convert_odom_to_state
  convert_vicon_to_state
  DESTINATION lib/${PROJECT_NAME}
)

# install Python executables
install(PROGRAMS
  scripts/map_odom_tf.py
  scripts/wait_for_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Tests
# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Manual solver benchmark (prints timings; not a gtest, not installed).
  add_executable(test_lbfgs_solver src/test/test_lbfgs_solver.cpp src/mighty/lbfgs_solver.cpp src/mighty/lbfgs_solver_utils.cpp)
  ament_target_dependencies(test_lbfgs_solver ${PLANNER_DEPS})

  ament_add_gtest(test_occ_grid_2d_unknown
    src/test/test_occ_grid_2d_unknown.cpp)
  ament_target_dependencies(test_occ_grid_2d_unknown
    rclcpp Eigen3 nav_msgs)

  # Tri-state 2D planning map + A* unknown pricing (2026-09; see map_util.hpp
  # buildMap2DFromOcc2D and graph_search.cpp getSucc).
  ament_add_gtest(test_map_2d_tristate
    src/test/test_map_2d_tristate.cpp)
  target_link_libraries(test_map_2d_tristate ${PCL_LIBRARIES})
  ament_target_dependencies(test_map_2d_tristate ${PLANNER_DEPS})

  ament_add_gtest(test_graph_search_unknown_cost
    src/test/test_graph_search_unknown_cost.cpp
    src/hgp/graph_search.cpp)
  target_link_libraries(test_graph_search_unknown_cost ${PCL_LIBRARIES})
  ament_target_dependencies(test_graph_search_unknown_cost ${PLANNER_DEPS})

  ament_add_gtest(test_frontier_detector
    src/test/test_frontier_detector.cpp
    src/mighty/frontier_detector.cpp)
  ament_target_dependencies(test_frontier_detector
    rclcpp Eigen3 nav_msgs)

  ament_add_gtest(test_frontier_manager
    src/test/test_frontier_manager.cpp
    src/mighty/frontier_detector.cpp
    src/mighty/frontier_manager.cpp)
  ament_target_dependencies(test_frontier_manager
    rclcpp Eigen3 nav_msgs)

  ament_add_gtest(test_gradient_check
    src/test/test_gradient_check.cpp
    src/mighty/lbfgs_solver.cpp
    src/mighty/lbfgs_solver_utils.cpp)
  ament_target_dependencies(test_gradient_check ${PLANNER_DEPS})

  ament_add_gtest(test_formation_cost
    src/test/test_formation_cost.cpp
    src/mighty/lbfgs_solver.cpp
    src/mighty/lbfgs_solver_utils.cpp)
  ament_target_dependencies(test_formation_cost ${PLANNER_DEPS})
endif()

ament_package()
