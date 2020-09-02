# cmake_minimum_required(VERSION 2.8.3)
# project(active_3d_planning_fiesta)

# find_package(catkin_simple REQUIRED COMPONENTS
#         roscpp
#         std_msgs
#         nav_msgs
#         visualization_msgs
#         )

# find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED)

# set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

# include_directories(
#     include
#     SYSTEM
#     ${PROJECT_SOURCE_DIR}/include
#     ${catkin_INCLUDE_DIRS}
#     ${Eigen3_INCLUDE_DIRS}
#     ${PCL_INCLUDE_DIRS}
# )

# catkin_simple(ALL_DEPS_REQUIRED)

# add_definitions(-std=c++11)

# catkin_package()

# cs_add_library(${PROJECT_NAME}
#         src/map/fiesta_interface.cpp
#         src/initialization/fiesta_package.cpp
#         )

# cs_install()
# cs_export()


cmake_minimum_required(VERSION 2.8.3)
project(active_3d_planning_fiesta)

find_package(catkin_simple REQUIRED)
#find_package(fiesta REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)

catkin_package(
        INCLUDE_DIRS include
)

cs_add_library(${PROJECT_NAME}
        src/map/fiesta_interface.cpp
        src/initialization/fiesta_package.cpp
        )

cs_install()
cs_export()
