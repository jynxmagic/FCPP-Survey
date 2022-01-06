cmake_minimum_required(VERSION 3.0.2)
project(full_coverage_path_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            base_local_planner
            costmap_2d
            nav_core
            pluginlib
            roscpp
            roslint
            rostest
            tf
        )

include_directories(
    include
    test/include
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        base_local_planner
        costmap_2d
        nav_core
        pluginlib
        roscpp
)

add_library(${PROJECT_NAME}
        src/common.cpp
        src/${PROJECT_NAME}.cpp
        src/spiral_stc.cpp
        src/uturn.cpp
        src/ant.cpp
        src/aco.cpp
        )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

install(TARGETS
            ${PROJECT_NAME}
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES fcpp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    nodes/coverage_progress
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

roslint_cpp()
