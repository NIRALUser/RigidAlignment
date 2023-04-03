cmake_minimum_required(VERSION 3.5)

project(RigidAlignment)

#-----------------------------------------------------------------------------
set(MODULE_NAME RigidAlignment)

#-----------------------------------------------------------------------------
# VTK
find_package(VTK REQUIRED)

# MeshLib
find_package(MeshLib REQUIRED)

# SlicerExecutionModel
find_package(SlicerExecutionModel REQUIRED)
include(${SlicerExecutionModel_USE_FILE})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#-----------------------------------------------------------------------------
set(${MODULE_NAME}_CATEGORY "" CACHE STRING "Module category")

set(MODULE_CATEGORY_CONFIG "")
if(NOT "${${MODULE_NAME}_CATEGORY}" STREQUAL "")
  set(MODULE_CATEGORY_CONFIG "<category>${${MODULE_NAME}_CATEGORY}</category>")
endif()
configure_file(
  ${MODULE_NAME}.xml.in
  ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}.xml
  @ONLY
  )

#-----------------------------------------------------------------------------
set(MODULE_INCLUDE_DIRECTORIES
  )

set(MODULE_SRCS
  RigidAlignmentImpl.cpp
  )

set(MODULE_TARGET_LIBRARIES
  Mesh
  VTK::CommonCore
  VTK::IOCore
  VTK::IOLegacy
  )

#-----------------------------------------------------------------------------
SEMMacroBuildCLI(
  NAME ${MODULE_NAME}
  CLI_XML_FILE ${CMAKE_CURRENT_BINARY_DIR}/${MODULE_NAME}.xml
  TARGET_LIBRARIES ${MODULE_TARGET_LIBRARIES}
  INCLUDE_DIRECTORIES ${MODULE_INCLUDE_DIRECTORIES}
  ADDITIONAL_SRCS ${MODULE_SRCS}
  EXECUTABLE_ONLY

  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}
  INSTALL_RUNTIME_DESTINATION ${INSTALL_RUNTIME_DESTINATION}
  INSTALL_LIBRARY_DESTINATION ${INSTALL_LIBRARY_DESTINATION}
  INSTALL_ARCHIVE_DESTINATION ${INSTALL_ARCHIVE_DESTINATION}
  )

#-----------------------------------------------------------------------------
if(BUILD_TESTING)
#  add_subdirectory(Testing)
endif()
