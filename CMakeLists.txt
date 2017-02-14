cmake_minimum_required(VERSION 2.8.3)
project(simplelayer)
find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            roscpp
            tf
            costmap_2d
            nav_core
            pluginlib
            base_local_planner
        )

find_package(Eigen REQUIRED)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS} include)

add_library(simple_layer src/simple_layer.cpp)

