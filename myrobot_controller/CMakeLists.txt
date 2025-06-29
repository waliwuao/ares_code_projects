cmake_minimum_required(VERSION 3.8)
project(myrobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

option(R1 "Build for R1 configuration" OFF)
option(R2 "Build for R2 configuration" OFF)

# 如果 FOR_R1 和 FOR_R2 都未被选中，则报错
if((NOT R1 AND NOT R2) OR (R1 AND R2))
  message(FATAL_ERROR "Must select one of the build configurations (R1 or R2)!")
endif()

ament_python_install_package(${PROJECT_NAME})

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(simple_controller src/simple_controller.cpp)
ament_target_dependencies(simple_controller rclcpp geometry_msgs std_msgs ${Eigen_LIBRARIES})

install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  simple_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/simple_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
