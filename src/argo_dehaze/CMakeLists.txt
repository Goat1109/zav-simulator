cmake_minimum_required(VERSION 3.8)
project(argo_dehaze)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

if(OpenMP_CXX_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Core Algorithm Library
add_library(adaptive_dcp_lib SHARED
  src/adaptive_dcp.cpp
)
target_link_libraries(adaptive_dcp_lib
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

# 1. Offline Dataset Benchmark Executable
add_executable(argo_dehaze_eval
  src/dehaze_eval.cpp
)
target_link_libraries(argo_dehaze_eval
  adaptive_dcp_lib
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

# 2. ROS 2 Real-time Dehazing Node
add_executable(argo_dehaze_node
  src/dehaze_node.cpp
)
ament_target_dependencies(argo_dehaze_node
  rclcpp
  sensor_msgs
  cv_bridge
  image_transport
)
target_link_libraries(argo_dehaze_node
  adaptive_dcp_lib
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

# 3. Interactive GUI Slider Tuner Executable
add_executable(argo_dehaze_tuner
  src/dehaze_tuner.cpp
)
target_link_libraries(argo_dehaze_tuner
  adaptive_dcp_lib
  ${OpenCV_LIBS}
  OpenMP::OpenMP_CXX
)

# Install rules
install(TARGETS
  adaptive_dcp_lib
  argo_dehaze_eval
  argo_dehaze_node
  argo_dehaze_tuner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
