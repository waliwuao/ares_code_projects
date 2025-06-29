cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(R1 "Build for R1 configuration" OFF)
option(R2 "Build for R2 configuration" OFF)

# 如果 FOR_R1 和 FOR_R2 都未被选中，则报错
if((NOT R1 AND NOT R2) OR (R1 AND R2))
  message(FATAL_ERROR "Must select one of the build configurations (R1 or R2)!")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch rviz  src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
