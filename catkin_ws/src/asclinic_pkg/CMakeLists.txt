cmake_minimum_required(VERSION 3.0.2)
project(asclinic_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  sensor_msgs
)

find_package(geometry_msgs REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

add_message_files(
  FILES
  ServoPulseWidth.msg
  TemplateMessage.msg
  LeftRightInt32.msg
  LeftRightFloat32.msg
  FiducialMarker.msg
  FiducialMarkerArray.msg
  PoseSeqs.msg
  PoseCovar.msg
  Covar.msg
  PlantDetection.msg
  PoseVelCovar.msg
  referenceVelocityPose.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES asclinic
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
# )

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(
#   include
#   ${catkin_INCLUDE_DIRS}
# )
include_directories(
  include
  src/drivers/src/
  #include/nodes
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/asclinic.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/asclinic_node.cpp)

# TEMPLATE CPP NODE:
add_executable(template_cpp_node                     src/nodes/template_cpp_node.cpp)
add_executable(template_cpp_node_minimal             src/nodes/template_cpp_node_minimal.cpp)
add_executable(template_gpio_event_triggered         src/nodes/template_gpio_event_triggered.cpp)
add_executable(template_gpio_polling                 src/nodes/template_gpio_polling.cpp)
add_executable(encoder_read_multi_threaded           src/nodes/encoder_read_multi_threaded.cpp)
add_executable(i2c_for_motors_and_servos             src/nodes/i2c_for_motors_and_servos.cpp
                                                     src/drivers/src/i2c_driver/i2c_driver.cpp
                                                     src/drivers/src/pololu_smc_g2/pololu_smc_g2.cpp
                                                     src/drivers/src/pca9685/pca9685.cpp)
add_executable(i2c_for_motors                        src/nodes/i2c_for_motors.cpp
                                                     src/drivers/src/i2c_driver/i2c_driver.cpp
                                                     src/drivers/src/pololu_smc_g2/pololu_smc_g2.cpp)
add_executable(i2c_for_servos                        src/nodes/i2c_for_servos.cpp
                                                     src/drivers/src/i2c_driver/i2c_driver.cpp
                                                     src/drivers/src/pca9685/pca9685.cpp)
add_executable(i2c_for_sensors                       src/nodes/i2c_for_sensors.cpp
                                                     src/drivers/src/i2c_driver/i2c_driver.cpp
                                                     src/drivers/src/vl53l1x/vl53l1x.cpp
                                                     src/drivers/src/vl53l1x/core/VL53L1X_api.c
                                                     src/drivers/src/vl53l1x/platform/vl53l1_platform.c)
add_executable(emulate_i2c_for_motors                src/nodes/emulate_i2c_for_motors.cpp)
add_executable(web_interface_connector               src/nodes/web_interface_connector.cpp)
add_executable(control_policy_skeleton               src/nodes/control_policy_skeleton.cpp)
add_executable(motor_publisher                      src/drivers/scripts/motor_publisher.cpp)
add_executable(take_photo                           src/drivers/scripts/take_photo.cpp)
add_executable(fun_publisher src/nodes/fun_publisher.cpp)
add_executable(odometer src/nodes/odometer.cpp)
add_executable(odomdelta src/nodes/odomdelta.cpp)
add_executable(odomtester src/nodes/odomtester.cpp)
add_executable(rotatetester src/nodes/rotatetester.cpp)
add_executable(rotatetesterdelta src/nodes/rotatetesterdelta.cpp)
# Add the new controller_node executable
add_executable(controller_node src/nodes/controller_node.cpp)

# Add the new velocity_PI_control node
add_executable(velocity_PI_control src/nodes/velocity_PI_control.cpp)

# Add the new trajectory_tracker node
add_executable(trajectory_tracker src/nodes/trajectory_tracker.cpp)
add_dependencies(trajectory_tracker asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_tracker ${catkin_LIBRARIES})

# Add the new trajectory_generator node
add_executable(trajectory_generator src/nodes/trajectory_generator.cpp)
add_dependencies(trajectory_generator asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_generator ${catkin_LIBRARIES})

# Add the new obstacledetect node
add_executable(obstacledetect src/nodes/obstacledetect.cpp)
add_dependencies(obstacledetect asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacledetect ${catkin_LIBRARIES})

# Add the new trajectory_generator_state_space node
add_executable(trajectory_generator_state_space src/nodes/trajectory_generator_state_space.cpp)
add_dependencies(trajectory_generator_state_space asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_generator_state_space ${catkin_LIBRARIES})
# Add the new trajectory_tracker_state_space node
add_executable(trajectory_tracker_state_space src/nodes/trajectory_tracker_state_space.cpp)
add_dependencies(trajectory_tracker_state_space asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_tracker_state_space ${catkin_LIBRARIES})
## Rename C++ executable with##out prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# TEMPLATE CPP NODE:
add_dependencies(template_cpp_node                     asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(template_cpp_node_minimal             asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(template_gpio_event_triggered         asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(template_gpio_polling                 asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(encoder_read_multi_threaded           asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(i2c_for_motors_and_servos             asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(i2c_for_motors                        asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(i2c_for_servos                        asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(i2c_for_sensors                       asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(emulate_i2c_for_motors                asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(web_interface_connector               asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(control_policy_skeleton               asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(take_photo                            asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})  
add_dependencies(motor_publisher                       asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})   
add_dependencies(fun_publisher                         asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(odometer                              asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(odomdelta                             asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(odomtester                            asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(rotatetester                          asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(rotatetesterdelta                     asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(controller_node                       asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(velocity_PI_control                   asclinic_pkg_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# TEMPLATE CPP NODE
target_link_libraries(template_cpp_node                     ${catkin_LIBRARIES})
target_link_libraries(template_cpp_node_minimal             ${catkin_LIBRARIES})
target_link_libraries(template_gpio_event_triggered         ${catkin_LIBRARIES} -lgpiod)
target_link_libraries(template_gpio_polling                 ${catkin_LIBRARIES} -lgpiod)
target_link_libraries(encoder_read_multi_threaded           ${catkin_LIBRARIES} -lgpiod)
target_link_libraries(i2c_for_motors_and_servos             ${catkin_LIBRARIES})
target_link_libraries(i2c_for_motors                        ${catkin_LIBRARIES})
target_link_libraries(i2c_for_servos                        ${catkin_LIBRARIES})
target_link_libraries(i2c_for_sensors                       ${catkin_LIBRARIES})
target_link_libraries(emulate_i2c_for_motors                ${catkin_LIBRARIES})
target_link_libraries(web_interface_connector               ${catkin_LIBRARIES})
target_link_libraries(control_policy_skeleton               ${catkin_LIBRARIES})
target_link_libraries(take_photo                            ${catkin_LIBRARIES})
target_link_libraries(motor_publisher                       ${catkin_LIBRARIES})
target_link_libraries(fun_publisher                         ${catkin_LIBRARIES})
target_link_libraries(odometer                              ${catkin_LIBRARIES})
target_link_libraries(odomdelta                             ${catkin_LIBRARIES})
target_link_libraries(odomtester                            ${catkin_LIBRARIES})
target_link_libraries(rotatetester                          ${catkin_LIBRARIES})
target_link_libraries(rotatetesterdelta                          ${catkin_LIBRARIES})
target_link_libraries(controller_node                       ${catkin_LIBRARIES})
target_link_libraries(velocity_PI_control                   ${catkin_LIBRARIES})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS src/drivers/scripts/encoder_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_asclinic.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
