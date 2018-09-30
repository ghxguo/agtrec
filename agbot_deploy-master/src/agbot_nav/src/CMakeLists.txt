cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(agbot_nav)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(agbot_nav new_ppcontroller.cpp utilities.h utilities.cpp UTM.cpp UTM.h)

target_link_libraries (agbot_nav Eigen3::Eigen)