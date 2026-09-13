cmake_minimum_required(VERSION 3.16)

set(SOURCES
# clsmmap
"src/clsmmap/ClsMMap.cpp"

# collision_detector
 "src/collision_detector/LayerBasedCollisionDetector.cpp"
 "src/collision_detector/NeighborBasedCollisionDetector.cpp"

# controller
 "src/controller/controller.cpp"
 "src/controller/spsc_slot_controller.cpp"

# cpp_helper_lib
"src/cpp_helper_lib/eigen_operator.cpp"

# detect3d
"src/detect3d/detect3d.cpp"
"src/detect3d/utils.cpp"

# edge_det
 "src/edge_det/transform.cpp"
 "src/edge_det/edge_utility.cpp"
 "src/edge_det/edge_detector.cpp"
 
# machine_collision
 "src/machine_collision/CircumcenterProcessor.cpp" 
 "src/machine_collision/machine_collision.cpp" 
 "src/machine_collision/MachineCollisionBase.cpp" 
 "src/machine_collision/MachineCollisionImmobileCuboid.cpp" 
 "src/machine_collision/MachineCollisionImmobileRoundCuboid.cpp" 
 "src/machine_collision/MachineCollisionImmobileRoundCuboid.cpp" 
 "src/machine_collision/MachineCollisionImmobileHexaPrism.cpp" 
 "src/machine_collision/MachineCollisionMobileCuboid.cpp" 
 "src/machine_collision/MachineConf.cpp" 

# octotree
 "src/octotree/OctoNode.cpp"
 "src/octotree/OctoTree.cpp"
 "src/octotree/VoxStats.cpp"

# scene
"src/scene/SceneDesc.cpp"

# error_mmap_writer
"src/error_mmap_writer/error_mmap_writer.cpp"

# ui_interface
"src/ui_interface/IFCliffInfo.cpp"
"src/ui_interface/status_mmap.cpp"
"src/ui_interface/ui_interface.cpp"
"src/ui_interface/vis_octotree.cpp"
"src/ui_interface/visualizer.cpp"
 )

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        OpenMP::OpenMP_CXX
        Open3D::Open3D
        nanoflann::nanoflann
        argus_eigen
        nlohmann_json::nlohmann_json
        argus_opencv
)

if(ARGUS_EXTERNAL_DEPS)
    add_dependencies(${PROJECT_NAME} ${ARGUS_EXTERNAL_DEPS})
endif()


install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION include)
