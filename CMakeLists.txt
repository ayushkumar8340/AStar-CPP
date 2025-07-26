cmake_minimum_required(VERSION 3.5)
project(planner)

find_package(OpenCV 4.1.1 REQUIRED)

add_library(custom_map SHARED src/custom_map.cpp)
target_link_libraries(custom_map PUBLIC ${OpenCV_LIBS} yaml-cpp)
target_include_directories(custom_map PUBLIC include)

add_library(planner_utils SHARED src/utils.cpp)
target_link_libraries(planner_utils PUBLIC ${OpenCV_LIBS})
target_include_directories(planner_utils PUBLIC include)

add_library(a_star SHARED src/a_star.cpp)
target_link_libraries(a_star PUBLIC custom_map planner_utils)

add_library(post_processing SHARED src/post_processing.cpp)
target_link_libraries(post_processing PUBLIC custom_map planner_utils)


########################################################################################
################## TEST SCRIPTS ########################################################

add_executable(test_planner src/tests/test_planner.cpp)
target_link_libraries(test_planner PUBLIC a_star post_processing)
