cmake_minimum_required(VERSION 3.8)
project(scalarpunk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# find dependencies
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


find_package(Qt5 REQUIRED COMPONENTS Widgets)
#find_package(Qt6 COMPONENTS Core Widgets REQUIRED)
#qt_standard_project_setup()

add_executable(scalarpunk
    main.cpp
    mainwindow.h
    mainwindow.cpp
    Scalar_punk_widget.h
    Scalar_punk_widget.cpp

)
target_link_libraries(scalarpunk ${Qt5Core_LIBRARIES})
target_link_libraries(scalarpunk ${Qt5Widgets_LIBRARIES})
ament_target_dependencies(scalarpunk rclcpp  std_msgs)


install(TARGETS
  scalarpunk
  DESTINATION lib/${PROJECT_NAME})

#---------------- END -----------------#

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
