cmake_minimum_required(VERSION 3.5)
project(planner)

find_package(OpenCV 4.1.1 REQUIRED)

add_library(custom_map SHARED src/custom_map.cpp)
target_link_libraries(custom_map PUBLIC ${OpenCV_LIBS} yaml-cpp)
target_include_directories(custom_map PUBLIC include)


