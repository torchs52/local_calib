cmake_minimum_required(VERSION 3.16)

set(SOURCES
# python wrapper
"src/target.cpp"
"src/collision_detector_module.cpp"
"src/controller_module.cpp"
"src/edge_det_module.cpp"
"src/machine_collision_module.cpp" 
"src/octotree_module.cpp"
"src/dataclass_module.cpp"
"src/SceneModule.cpp"
"src/status_mmap_module.cpp"
"src/error_mmap_writer_module.cpp"
"src/ui_interface_module.cpp"
"src/visualizer_module.cpp"
"src/detect3d_module.cpp"
"src/spsc_slot_controller_module.cpp"
 )

pybind11_add_module(pybind ${SOURCES})
set_target_properties(pybind PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
target_include_directories(pybind PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(pybind
    PRIVATE
        OpenMP::OpenMP_CXX
        argus_synchro_lib
        argus_eigen
        nlohmann_json::nlohmann_json
        argus_opencv
)
