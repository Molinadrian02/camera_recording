cmake_minimum_required(VERSION 3.5)
project(hk_camera_interfaces)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TakePhoto.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
