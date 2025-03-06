cmake_minimum_required(VERSION 3.16)
project(2DRoboticArmSimulation)

set(CMAKE_CXX_STANDARD 20)

find_package(SFML 2.5 REQUIRED COMPONENTS graphics window system)


add_executable(2DRoboticArmSimulation main.cpp)
target_link_libraries(2DRoboticArmSimulation sfml-graphics sfml-window sfml-system)


