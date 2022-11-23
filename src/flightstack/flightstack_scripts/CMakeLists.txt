cmake_minimum_required(VERSION 3.0.2)
project(flightstack_scripts)

find_package(catkin REQUIRED)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES chb_test
    #  CATKIN_DEPENDS other_catkin_pkg
    #  DEPENDS system_lib
    )

add_custom_target(copy-test-makefile ALL DEPENDS /usr/bin/fs_dev1 /usr/bin/fs_dev2 /usr/bin/fs_op)
add_custom_command(
  OUTPUT /usr/bin/fs_dev1
  COMMAND sudo ln -s
          ${CMAKE_SOURCE_DIR}/launch_scripts/flightstack_dev1.sh
          /usr/bin/fs_dev1
)
add_custom_command(
  OUTPUT /usr/bin/fs_dev2
  COMMAND sudo ln -s
          ${CMAKE_SOURCE_DIR}/launch_scripts/flightstack_dev2.sh
          /usr/bin/fs_dev2
)
add_custom_command(
  OUTPUT /usr/bin/fs_op
  COMMAND sudo ln -s
          ${CMAKE_SOURCE_DIR}/launch_scripts/flightstack_op.sh
          /usr/bin/fs_op
)
