cmake_minimum_required(VERSION 3.8)
project(ship_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(ShipMotionPlugin SHARED
  src/ShipMotionPlugin.cc
)

target_include_directories(ShipMotionPlugin PUBLIC
  include/
)

target_link_libraries(ShipMotionPlugin
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
)

install(TARGETS ShipMotionPlugin
  DESTINATION lib/${PROJECT_NAME}
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

install(DIRECTORY
  worlds
  models
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
