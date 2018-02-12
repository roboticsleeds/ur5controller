cmake_minimum_required(VERSION 2.4.6)
project(ur5controller)

include(CheckCXXSourceCompiles)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    openrave_catkin
    std_msgs
    robotiq_c_model_control
    trajectory_msgs
    actionlib
    actionlib_msgs
    control_msgs
)


include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
    ${PROJECT_NAME}/include
)

link_directories(
    ${catkin_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        robotiq_c_model_control
        std_msgs
        trajectory_msgs
        actionlib
        actionlib_msgs
        control_msgs
    DEPENDS
        boost
        openrave
)

include_directories(include/${PROJECT_NAME})


# Helper library that implements core functionality.
add_library(${PROJECT_NAME} SHARED
    src/robotiqcontroller.cpp
    src/ur5controller.cpp
)

# Stub library that registers the plugins with OpenRAVE.
openrave_plugin(${PROJECT_NAME}_plugin
    src/ur5controller_interface.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
