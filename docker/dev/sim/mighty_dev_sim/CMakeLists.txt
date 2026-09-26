cmake_minimum_required(VERSION 3.8)
project(mighty_dev_sim)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Default build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynus_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Integrates mighty's goal setpoints into a state + TF + odometry (a perfect UAV).
add_executable(fake_sim src/fake_sim.cpp)
target_include_directories(fake_sim PRIVATE ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(fake_sim rclcpp dynus_interfaces geometry_msgs nav_msgs tf2 tf2_ros visualization_msgs)

install(TARGETS fake_sim DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
