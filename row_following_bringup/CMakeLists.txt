cmake_minimum_required(VERSION 3.8)
project(row_following_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/row_follow.py
  scripts/navsatfix2tf.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME})

ament_package()
