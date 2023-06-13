cmake_minimum_required(VERSION 3.5)
project(dynamixel_wrapper)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# resolve path
include_directories(include/dynamixel_wrapper)
set(HEADER_FILES include/dynamixel_wrapper/dynamixel_handle.hpp)
add_library(dynamixel_wrapper src/dynamixel_handle.cpp ${HEADER_FILES})

ament_target_dependencies(dynamixel_wrapper
  dynamixel_sdk
)

# install library
install(
  DIRECTORY include/dynamixel_wrapper
  DESTINATION include
)

install(
  TARGETS dynamixel_wrapper
  EXPORT dynamixel_wrapper
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(dynamixel_wrapper HAS_LIBRARY_TARGET)
ament_package()
