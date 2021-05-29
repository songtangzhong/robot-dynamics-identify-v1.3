#
cmake_minimum_required(VERSION 3.0)

project(least_square_estimate)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_executable(${PROJECT_NAME} 
  src/least_square_estimate.cpp
  src/least_square.cpp
)
target_include_directories(${PROJECT_NAME}
  PRIVATE
  include 
  /usr/include
  /usr/local/MatlabR2019b/extern/include
)
target_link_libraries(${PROJECT_NAME} 
  librobotdynamics.so 
  libm.so
  /usr/local/MatlabR2019b/bin/glnxa64/libmat.so 
  /usr/local/MatlabR2019b/bin/glnxa64/libmx.so
)




