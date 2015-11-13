project(RigidAlignment)
cmake_minimum_required(VERSION 2.8)

find_package(SlicerExecutionModel REQUIRED)

include(${SlicerExecutionModel_USE_FILE})
include(${GenerateCLP_USE_FILE})
include_directories(/System/Library/Frameworks/Accelerate.framework/Headers)
include_directories(Mesh)
include_directories( ${CMAKE_SOURCE_DIR} ${CMAKE_BINARY_DIR} )

#Convert my XML into a C/C++ file->Creates GRccmTestProjectCLP.h in the binary directory
generateCLP(RIGID_WRAPPER_SRC wrapper/RigidWrapper.xml)

# MESH library
set(MESH_LIB_DIR /NIRAL/work/ilwoolyu/MeshLib)
include_directories(${MESH_LIB_DIR}/include ${MESH_LIB_DIR}/include/Util)
link_directories(${MESH_LIB_DIR}/lib)

set (Registration_SOURCES src/RigidAlignment.cpp)
include_directories(src)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
add_executable(RigidAlignment ${Registration_SOURCES} wrapper/RigidWrapper.cpp ${RIGID_WRAPPER_SRC})
target_link_libraries(RigidAlignment Mesh)
