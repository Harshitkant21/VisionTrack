# # a separate CMake file for building tests

# # Minimum CMake version
cmake_minimum_required(VERSION 3.10)
project(VisionTrackTests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/util
    ${OpenCV_INCLUDE_DIRS}
)

# Add test executables
add_executable(test_config test_config.cpp ${CMAKE_SOURCE_DIR}/src/config.cpp)
add_executable(test_detection test_detection.cpp ${CMAKE_SOURCE_DIR}/src/core/detection.cpp)
add_executable(test_tracking test_tracking.cpp test_tracking.cpp ${CMAKE_SOURCE_DIR}/src/core/tracking.cpp ${CMAKE_SOURCE_DIR}/src/core/hungarian.cpp)
add_executable(test_alerts test_alerts.cpp ${CMAKE_SOURCE_DIR}/src/util/alerts.cpp)

# Link GTest (built from subdirectory) + OpenCV
foreach(test_target test_detection test_tracking test_alerts test_config)
    target_link_libraries(${test_target}
        PRIVATE
        gtest
        gtest_main
        ${OpenCV_LIBS}
    )
    add_test(NAME ${test_target} COMMAND ${test_target})
endforeach()
