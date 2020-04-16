cmake_minimum_required(VERSION 2.8.3)
project(elfin_driver)
add_definitions(-std=c++11)
find_package(catkin REQUIRED COMPONENTS industrial_robot_client simple_message)

catkin_package(
  CATKIN_DEPENDS industrial_robot_client
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication

add_executable(${PROJECT_NAME}_robot_state
  src/elfin_robot_status_relay_handler.cpp
  src/elfin_robot_joint_relay_handler.cpp
  src/elfin_robot_state_interface.cpp
  src/elfin_robot_state_node.cpp
  src/elfin_utils.cpp)
target_link_libraries(${PROJECT_NAME}_robot_state
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(
  ${PROJECT_NAME}_robot_state
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}_robot_state PREFIX "")

add_executable(${PROJECT_NAME}_motion_download_interface
  src/elfin_joint_trajectory_downloader.cpp
  src/elfin_joint_downloader_node.cpp
  src/elfin_utils.cpp)
target_link_libraries(${PROJECT_NAME}_motion_download_interface
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_download_interface
  PROPERTIES OUTPUT_NAME motion_download_interface PREFIX "")

# Unit testing requires a physical robot. Commented for CI.
#
#if(CATKIN_ENABLE_TESTING)
#    catkin_add_gtest(utest_elfin_driver test/elfin_driver_gtest.cpp)
#    target_link_libraries(utest_elfin_driver  
#	  industrial_robot_client
#	  simple_message
#	${catkin_LIBRARIES})
#    install(
#	FILES test/elfin_launch.sh
#	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )
#endif()


install(
  TARGETS
    ${PROJECT_NAME}_robot_state
    ${PROJECT_NAME}_motion_download_interface
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
  install(
    DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
