cmake_minimum_required(VERSION 3.8)
project(decision_maker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads REQUIRED)


include_directories(
  include
  ${CURL_INCLUDE_DIRS}
)

# Create the node executable
add_executable(decision_maker_node 
  src/decision_maker_node.cpp
)


target_link_libraries(decision_maker_node
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
  Threads::Threads
)

ament_target_dependencies(decision_maker_node
  rclcpp
  std_msgs
)

install(TARGETS decision_maker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
