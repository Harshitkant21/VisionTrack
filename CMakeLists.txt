# This will be the main CMake file

cmake_minimum_required(VERSION 3.10)
project(VisionTrack)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable Testing
enable_testing()

set(OpenCV_DIR "D:/VisionTrack/include/install")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_DIR} include src/core src/utils src/ui)

# Add GoogleTest as a subdirectory 
add_subdirectory(include/googletest)

# Add the tests directory
add_subdirectory(tests)

# Add executable for the main application
add_executable(VisionTrack src/main.cpp src/core/detection.cpp src/core/tracking.cpp src/core/hungarian.cpp src/util/alerts.cpp src/config.cpp)


target_link_libraries(VisionTrack ${OpenCV_LIBS})

