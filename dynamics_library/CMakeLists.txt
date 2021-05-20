#
cmake_minimum_required(VERSION 3.0)

project(robotdynamics)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_library(${PROJECT_NAME} SHARED 
  src/robot_model/robot_model.cpp
  src/robot_dynamics/robot_dynamics.cpp
  src/dynamics_identify/dynamics_identify.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE 
  include
)

install(DIRECTORY include/
  DESTINATION /usr/include
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION /usr/lib
)





