cmake_minimum_required(VERSION 3.8)
project(whisper_asr)


# Add the path to the cmake/ folder in whisper.cpp
set(WHISPER_CPP_DIR "/root/ros2_ws/src/speech-llm-speech/whisper.cpp")
set(CMAKE_MODULE_PATH "${WHISPER_CPP_DIR}/cmake" ${CMAKE_MODULE_PATH})
# message("CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
# message("WHISPER_CPP_DIR: ${WHISPER_CPP_DIR}")

# Path to the whisper.cpp build directory where libwhisper.a is generated
set(WHISPER_CPP_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../whisper.cpp/build")

# Add whisper.cpp source files
set(WHISPER_CPP_SOURCES
    ${WHISPER_CPP_DIR}/src/whisper.cpp  # Adjusted to point to the actual file
    ${WHISPER_CPP_DIR}/ggml/     # Include ggml core if required
    ${WHISPER_CPP_DIR}/ggml/include     # Include ggml include manually if required
    ${WHISPER_CPP_DIR}/cmake     # Include cmake after building if required
)


# Ensure CMake knows about this path
link_directories(${WHISPER_CPP_BUILD_DIR})


# Add include directories
include_directories(
    ${WHISPER_CPP_DIR}/include
    ${WHISPER_CPP_DIR}/ggml
    ${WHISPER_CPP_DIR}/ggml/include     # Include ggml core if required
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(whisper_asr_node src/whisper_asr_node.cpp ${WHISPER_CPP_DIR}/src)

# Link whisper library
target_link_libraries(whisper_asr_node ${WHISPER_CPP_BUILD_DIR}/src/libwhisper.so)  # Link the static library

ament_target_dependencies(whisper_asr_node
  rclcpp
  std_msgs
)

install(TARGETS
  whisper_asr_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
