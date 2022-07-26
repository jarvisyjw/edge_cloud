# cmake needs this line
cmake_minimum_required(VERSION 2.8)

# Define project name
project(optical_flow)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV 3.4.15 REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")


if(CMAKE_VERSION VERSION_LESS "2.8.11")
  # Add OpenCV headers location to your include paths
  include_directories(${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include)
endif()

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/ORBextractor.cc
)

# Declare the executable target built from your sources
add_executable(cal_optical_flow src/optical_flow.cpp)
add_executable(main src/main.cpp)

# Link your application with OpenCV libraries
target_link_libraries(cal_optical_flow PRIVATE ${OpenCV_LIBS}
${PROJECT_NAME})
target_link_libraries(main PRIVATE ${OpenCV_LIBS}
${PROJECT_NAME})
