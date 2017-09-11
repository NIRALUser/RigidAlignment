add_library(RigidAlignment_SOURCES 
        STATIC
        RigidAlignment.cpp)
target_link_libraries(RigidAlignment_SOURCES ${Mesh_LIBRARIES})