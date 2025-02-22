cmake_minimum_required(VERSION 3.22)
project(rushbott_moveit_config)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  moveit_msgs
  std_srvs
  rclcpp
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(servo_keyboard_input teleop/servo_keyboard_input.cpp)
target_include_directories(servo_keyboard_input PUBLIC include)
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS servo_keyboard_input
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/rushbott_moveit_config)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})


ament_package()