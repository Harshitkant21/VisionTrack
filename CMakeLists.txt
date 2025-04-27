# This will be the main CMake file

cmake_minimum_required(VERSION 3.10)
project(VisionTrack)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(OpenCV_DIR "D:/Projects/VisionTrack/include/install")
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_DIR} include src/core src/utils src/ui)

# add_executable(VisionTrack src/main.cpp src/core/detection.cpp src/core/tracking.cpp src/core/classifier.cpp src/utils/video.cpp src/utils/alerts.cpp src/ui/interface.cpp)
add_executable(VisionTrack src/main.cpp src/core/detection.cpp src/core/tracking.cpp src/config.cpp)

# add_subdirectory(tests)

target_link_libraries(VisionTrack ${OpenCV_LIBS})

