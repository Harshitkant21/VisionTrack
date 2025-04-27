# a separate CMake file for building tests

# # Minimum CMake version
# cmake_minimum_required(VERSION 3.10)

# # Project name for tests
# project(VisionTrackTests)

# # Find OpenCV
# find_package(OpenCV REQUIRED)
# include_directories(${OpenCV_INCLUDE_DIRS})

# # Include the project's include directory
# include_directories(${CMAKE_SOURCE_DIR}/include)

# # Add test executable for classifier
# add_executable(test_classifier test_classifier.cpp ${CMAKE_SOURCE_DIR}/src/core/classifier.cpp)
# target_link_libraries(test_classifier PRIVATE ${OpenCV_LIBS})