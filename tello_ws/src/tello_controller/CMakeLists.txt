cmake_minimum_required(VERSION 3.8)
project(tello_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(src include)

add_library(socket STATIC 
network/socket.cpp 
network/socket.hpp)

add_library(udpsocket STATIC
    network/udpsocket.cpp
    network/udpsocket.hpp
)

add_library(tcpsocket STATIC 
  network/tcpsocket.cpp 
  network/tcpsocket.hpp
)

target_link_libraries(udpsocket socket)
target_link_libraries(tcpsocket socket)

install(
  TARGETS udpsocket tcpsocket
)


add_executable(controller src/controller.cpp)
ament_target_dependencies(controller
  rclcpp
  std_msgs
  geometry_msgs
)

add_library(skills_lib SHARED 
src/skills.cpp src/droneskills.cpp src/skillsmain.cpp)
ament_target_dependencies(skills_lib  rclcpp geometry_msgs std_msgs)
add_executable(skills src/skills.cpp src/droneskills.cpp src/skillsmain.cpp)
target_link_libraries(skills skills_lib)
ament_target_dependencies(skills
  rclcpp
  geometry_msgs
  std_msgs
)

add_executable(teste src/teste.cpp)
target_link_libraries(teste skills_lib)
ament_target_dependencies(teste 
  rclcpp
  std_msgs
  geometry_msgs

)


target_link_libraries(controller tcpsocket)
target_link_libraries(controller udpsocket)

install(TARGETS
  skills
  controller
  teste
  EXPORT tcpsocket DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
  EXPORT udpsocket DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS skills_lib 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
