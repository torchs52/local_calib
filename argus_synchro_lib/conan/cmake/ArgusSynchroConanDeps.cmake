# ArgusSynchroConanDeps.cmake — wire Conan-generated packages to argus CMake targets.

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nanoflann REQUIRED)
find_package(Open3D CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pybind11 CONFIG REQUIRED)

add_library(argus_eigen INTERFACE)
target_link_libraries(argus_eigen INTERFACE Eigen3::Eigen)

add_library(argus_opencv INTERFACE)
if(TARGET opencv::opencv)
    target_link_libraries(argus_opencv INTERFACE opencv::opencv)
elseif(TARGET OpenCV::opencv_core AND TARGET OpenCV::opencv_calib3d)
    target_link_libraries(argus_opencv INTERFACE OpenCV::opencv_core OpenCV::opencv_calib3d)
else()
    message(FATAL_ERROR "OpenCV Conan targets not found (expected opencv::opencv or OpenCV::opencv_*)")
endif()

message(STATUS "Conan deps: Open3D ${Open3D_VERSION}, OpenCV found, pybind11 ${pybind11_VERSION}")
