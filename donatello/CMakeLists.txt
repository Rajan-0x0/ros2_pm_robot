cmake_minimum_required(VERSION 3.5)
project(donatello)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-stdlib=libstdc++)
endif()

# --------------------------------------------------
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtle_skillset_interfaces REQUIRED)
find_package(turtle_skillset REQUIRED)

# TODO
find_package(std_srvs REQUIRED)
find_package(turtlesim REQUIRED)

# --------------------------------------------------
include_directories(include)

# --------------------------------------------------------------------
add_library(donatello
  src/Node.cpp
)

target_link_libraries(donatello)
ament_target_dependencies(donatello
  rclcpp std_msgs turtle_skillset_interfaces turtle_skillset
  std_srvs turtlesim # TODO
)

install(DIRECTORY include/
  DESTINATION include)

install(
  TARGETS donatello
  EXPORT donatello
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# --------------------------------------------------------------------
# DEBUG
add_library(donatello_debug
  src/Node.cpp
)

target_compile_definitions(donatello_debug PUBLIC SKILLSET_DEBUG_MODE)
target_link_libraries(donatello_debug)
ament_target_dependencies(donatello_debug
  rclcpp std_msgs turtle_skillset_interfaces turtle_skillset
  std_srvs turtlesim # TODO
)

install(
  TARGETS donatello_debug
  EXPORT donatello_debug
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# --------------------------------------------------------------------
add_executable(donatello_node src/donatello_node.cpp)
target_link_libraries(donatello_node
  donatello
)
install(TARGETS
  donatello_node
  DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------------------
# DEBUG
add_executable(donatello_node_debug src/donatello_node.cpp)
target_link_libraries(donatello_node_debug
  donatello_debug
)
install(TARGETS
  donatello_node_debug
  DESTINATION lib/${PROJECT_NAME})

# --------------------------------------------------------------------
set(PROJECT_LIBS donatello)

ament_export_targets(donatello)

# --------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

