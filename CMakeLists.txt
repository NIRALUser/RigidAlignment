project(RigidWrapper)
cmake_minimum_required(VERSION 2.8)

find_package(SlicerExecutionModel REQUIRED)
find_package(MeshLib REQUIRED)
find_package(VTK REQUIRED)

include(${VTK_USE_FILE})
include(${SlicerExecutionModel_USE_FILE})
include(${GenerateCLP_USE_FILE})

include_directories(Mesh)
include_directories( ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} )

include_directories(src)
include_directories(wrapper)
add_subdirectory(wrapper)
add_subdirectory(src)

