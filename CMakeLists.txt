cmake_minimum_required(VERSION 3.14)
project(learning_udp)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost COMPONENTS system thread program_options REQUIRED)

add_executable(udp_robot udp_robot.cpp)
target_link_libraries(udp_robot
        ${Boost_LIBRARIES})

