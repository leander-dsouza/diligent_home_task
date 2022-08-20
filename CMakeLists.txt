cmake_minimum_required(VERSION 3.0.2)
project(diligent_home_task)

find_package(catkin REQUIRED)

catkin_package()
include_directories()

catkin_python_setup()

catkin_install_python(PROGRAMS
  scripts/polygonal_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests)
endif()
