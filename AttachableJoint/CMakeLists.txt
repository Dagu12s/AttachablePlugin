cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

find_package(ignition-cmake2 REQUIRED)

project(AttachableJoint)

find_package(ignition-physics2 REQUIRED)
set(IGN_MSGS_VER ${ignition-physics2_VERSION_MAJOR})

find_package(ignition-common3 REQUIRED)
set(IGN_MSGS_VER ${ignition-common3_VERSION_MAJOR})




find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo3 REQUIRED)
add_library(AttachableJoint SHARED AttachableJoint.cc AttachableJoint.hh)
set_property(TARGET AttachableJoint PROPERTY CXX_STANDARD 17)
target_link_libraries(AttachableJoint
  PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
  PRIVATE ignition-gazebo3::ignition-gazebo3
  PRIVATE ignition-common3::ignition-common3
  PRIVATE ignition-physics2::ignition-physics2
  )


