cmake_minimum_required(VERSION 3.10)
project(multi_drone_payload_lifting)

find_package(gz-cmake3 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-transport13 REQUIRED)

add_library(cable_plugin SHARED src/cable.cpp)

target_link_libraries(cable_plugin
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
)

# Install the plugin .so into the build folder directly
set_target_properties(cable_plugin PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
