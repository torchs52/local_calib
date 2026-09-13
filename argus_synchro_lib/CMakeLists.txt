cmake_minimum_required(VERSION 3.16)
project(argus_synchro_lib VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYBIND11_FINDPYTHON ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(ARGUS_OPENCV_BUILD_LIST "core,calib3d")

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(NOT ipo_supported)
    message(FATAL_ERROR "IPO/LTO is required but not supported: ${ipo_error}")
endif()
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

if(MSVC)
    message(FATAL_ERROR "This fixed build configuration requires -march=native, which is unsupported on MSVC.")
endif()
add_compile_options($<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:Release>>:-march=native>)
add_compile_options($<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CONFIG:Release>>:-funroll-loops>)

find_package(pybind11 3.0 CONFIG REQUIRED)
if (pybind11_FOUND)
    message(STATUS "pybind11 was found: ${pybind11_VERSION} ${pybind11_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "pybind11 was not found. Ensure that it is installed and accessible.")
endif()

include(FetchContent)
include(ExternalProject)
include(CMakeParseArguments)
include(ProcessorCount)

ProcessorCount(ARGUS_EXTERNAL_BUILD_JOBS)

function(argus_register_external_project target_name)
    set(options)
    set(oneValueArgs GIT_REPOSITORY GIT_TAG SOURCE_DIR BINARY_DIR INSTALL_DIR BYPRODUCT)
    set(multiValueArgs CMAKE_ARGS)
    cmake_parse_arguments(ARGUS_EXT "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(NOT ARGUS_EXT_SOURCE_DIR OR NOT ARGUS_EXT_BINARY_DIR OR NOT ARGUS_EXT_INSTALL_DIR)
        message(FATAL_ERROR "argus_register_external_project: required arguments are missing for ${target_name}.")
    endif()

    set(_argus_ext_download_args)
    if(ARGUS_EXT_GIT_REPOSITORY OR ARGUS_EXT_GIT_TAG)
        if(NOT ARGUS_EXT_GIT_REPOSITORY OR NOT ARGUS_EXT_GIT_TAG)
            message(FATAL_ERROR "argus_register_external_project: both GIT_REPOSITORY and GIT_TAG are required when using Git for ${target_name}.")
        endif()
        list(APPEND _argus_ext_download_args
            GIT_REPOSITORY ${ARGUS_EXT_GIT_REPOSITORY}
            GIT_TAG ${ARGUS_EXT_GIT_TAG}
            GIT_SHALLOW TRUE
            GIT_PROGRESS FALSE
            UPDATE_COMMAND ""
        )
    else()
        if(NOT EXISTS "${ARGUS_EXT_SOURCE_DIR}/CMakeLists.txt")
            message(FATAL_ERROR "argus_register_external_project: local source is not ready for ${target_name}: ${ARGUS_EXT_SOURCE_DIR}")
        endif()
        list(APPEND _argus_ext_download_args
            DOWNLOAD_COMMAND ""
            UPDATE_COMMAND ""
        )
    endif()

    ExternalProject_Add(
        ${target_name}
        ${_argus_ext_download_args}
        SOURCE_DIR ${ARGUS_EXT_SOURCE_DIR}
        BINARY_DIR ${ARGUS_EXT_BINARY_DIR}
        CMAKE_ARGS
            -DCMAKE_INSTALL_PREFIX=${ARGUS_EXT_INSTALL_DIR}
            -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
            -DBUILD_SHARED_LIBS=ON
            ${ARGUS_EXT_CMAKE_ARGS}
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --parallel ${ARGUS_EXTERNAL_BUILD_JOBS}
        INSTALL_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --target install --config Release --parallel ${ARGUS_EXTERNAL_BUILD_JOBS}
        BUILD_BYPRODUCTS ${ARGUS_EXT_BYPRODUCT}
    )

    set_property(GLOBAL APPEND PROPERTY ARGUS_EXTERNAL_DEPS_PROP ${target_name})
endfunction()

# Eigen (header-only)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS FALSE
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/../3rdparty/eigen
)
FetchContent_GetProperties(eigen)
if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
endif()
add_library(argus_eigen INTERFACE)
target_include_directories(argus_eigen INTERFACE ${eigen_SOURCE_DIR})

# nlohmann/json (header-only)
set(JSON_BuildTests OFF CACHE BOOL "Disable nlohmann/json tests" FORCE)
set(JSON_Install OFF CACHE BOOL "Disable nlohmann/json install" FORCE)
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS FALSE
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/../3rdparty/json
)
FetchContent_MakeAvailable(json)

# nanoflann (header-only)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "Disable nanoflann examples" FORCE)
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "Disable nanoflann tests" FORCE)
FetchContent_Declare(
    nanoflann
    GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
    GIT_TAG v1.9.0
    GIT_SHALLOW TRUE
    GIT_PROGRESS FALSE
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/../3rdparty/nanoflann
)
FetchContent_MakeAvailable(nanoflann)

find_package(OpenMP REQUIRED)
if(OpenMP_FOUND)
    message(STATUS "OpenMP was found: ${OpenMP_VERSION} ${OpenMP_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenMP was not found. Please make sure it is installed and accessible.")
endif()

# 実行環境判定
set(ON_JETSON OFF)
if(CMAKE_SYSTEM_PROCESSOR)
    string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _proc)
    if(_proc MATCHES "aarch64|arm64")
        if(EXISTS /etc/nv_tegra_release)
            set(ON_JETSON ON)
        endif()
    endif()
endif()

# Open3D
if(ON_JETSON AND EXISTS ${PROJECT_SOURCE_DIR}/../3rdparty/open3d_jetson)
    set(OPEN3D_SRC_DIR ${PROJECT_SOURCE_DIR}/../3rdparty/open3d_jetson)
else()
    set(OPEN3D_SRC_DIR ${PROJECT_SOURCE_DIR}/../3rdparty/open3d)
endif()
set(OPEN3D_LOCAL_SOURCE_READY OFF)
if(EXISTS ${OPEN3D_SRC_DIR}/CMakeLists.txt)
    set(OPEN3D_LOCAL_SOURCE_READY ON)
endif()
set(OPEN3D_INSTALL_DIR ${OPEN3D_SRC_DIR}/install)
set(OPEN3D_SHARED_LIB ${OPEN3D_INSTALL_DIR}/lib/libOpen3D.so)
set(OPEN3D_BUILD_DIR ${CMAKE_BINARY_DIR}/_external/open3d/build)
file(MAKE_DIRECTORY ${OPEN3D_INSTALL_DIR}/include)
file(MAKE_DIRECTORY ${OPEN3D_INSTALL_DIR}/lib)
file(MAKE_DIRECTORY ${OPEN3D_BUILD_DIR})

find_package(Open3D QUIET CONFIG PATHS ${OPEN3D_INSTALL_DIR} NO_DEFAULT_PATH)
if(NOT Open3D_FOUND)
    if(ON_JETSON)
        message(FATAL_ERROR "Open3D was not found in ${OPEN3D_INSTALL_DIR}. On Jetson, prebuilt Open3D is required (build with scripts/open3d/docker/docker_build.sh).")
    endif()

    set(OPEN3D_CMAKE_ARGS
        -DBUILD_PYTHON_MODULE=OFF
        -DBUILD_UNIT_TESTS=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_BENCHMARKS=OFF
        -DWITH_OPENMP=ON
        -DBUILD_GUI=OFF
        -DBUILD_WEBRTC=OFF
        -DBUILD_JUPYTER_EXTENSION=OFF
        -DBUILD_LIBREALSENSE=OFF
        -DBUILD_AZURE_KINECT=OFF
        -DWITH_IPP=OFF
        -DBUILD_ISPC_MODULE=OFF
        -DUSE_SYSTEM_TBB=ON
        -DUSE_BLAS=OFF
        -DBUILD_CUDA_MODULE=OFF
        # -DBUILD_VTK_FROM_SOURCE=ON
    )

    if(OPEN3D_LOCAL_SOURCE_READY)
        message(STATUS "Using local Open3D source: ${OPEN3D_SRC_DIR}")
        argus_register_external_project(
            open3d_external
            SOURCE_DIR ${OPEN3D_SRC_DIR}
            BINARY_DIR ${OPEN3D_BUILD_DIR}
            INSTALL_DIR ${OPEN3D_INSTALL_DIR}
            BYPRODUCT ${OPEN3D_SHARED_LIB}
            CMAKE_ARGS ${OPEN3D_CMAKE_ARGS}
        )
    else()
        message(STATUS "Local Open3D source was not found at ${OPEN3D_SRC_DIR}. Fetching from GitHub.")
        argus_register_external_project(
            open3d_external
            GIT_REPOSITORY https://github.com/isl-org/Open3D.git
            GIT_TAG v0.19.0
            SOURCE_DIR ${OPEN3D_SRC_DIR}
            BINARY_DIR ${OPEN3D_BUILD_DIR}
            INSTALL_DIR ${OPEN3D_INSTALL_DIR}
            BYPRODUCT ${OPEN3D_SHARED_LIB}
            CMAKE_ARGS ${OPEN3D_CMAKE_ARGS}
        )
    endif()

    if(NOT TARGET Open3D::Open3D)
        add_library(Open3D::Open3D SHARED IMPORTED GLOBAL)
    endif()
    set_target_properties(Open3D::Open3D PROPERTIES
        IMPORTED_LOCATION ${OPEN3D_SHARED_LIB}
        INTERFACE_INCLUDE_DIRECTORIES ${OPEN3D_INSTALL_DIR}/include
    )
endif()

# OpenCV
if(ON_JETSON AND EXISTS ${PROJECT_SOURCE_DIR}/../3rdparty/opencv_jetson)
    set(OPENCV_SRC_DIR ${PROJECT_SOURCE_DIR}/../3rdparty/opencv_jetson)
else()
    set(OPENCV_SRC_DIR ${PROJECT_SOURCE_DIR}/../3rdparty/opencv)
endif()
set(OPENCV_INSTALL_DIR ${OPENCV_SRC_DIR}/install)
set(OPENCV_BUILD_BYPRODUCT ${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so)
set(OPENCV_BUILD_DIR ${CMAKE_BINARY_DIR}/_external/opencv/build)
file(MAKE_DIRECTORY ${OPENCV_BUILD_DIR})

find_package(OpenCV QUIET CONFIG PATHS ${OPENCV_INSTALL_DIR} NO_DEFAULT_PATH)
if(NOT OpenCV_FOUND)
    if(ON_JETSON)
        message(STATUS "OpenCV was not found in ${OPENCV_INSTALL_DIR}. Building OpenCV from source on Jetson.")
    endif()

    argus_register_external_project(
        opencv_external
        GIT_REPOSITORY https://github.com/opencv/opencv.git
        GIT_TAG 4.13.0
        SOURCE_DIR ${OPENCV_SRC_DIR}
        BINARY_DIR ${OPENCV_BUILD_DIR}
        INSTALL_DIR ${OPENCV_INSTALL_DIR}
        BYPRODUCT ${OPENCV_BUILD_BYPRODUCT}
        CMAKE_ARGS
            -DBUILD_TESTS=OFF
            -DBUILD_PERF_TESTS=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_DOCS=OFF
            -DBUILD_opencv_apps=OFF
            -DBUILD_opencv_python3=OFF
            -DBUILD_opencv_world=OFF
            -DBUILD_opencv_highgui=OFF
            -DBUILD_LIST=${ARGUS_OPENCV_BUILD_LIST}
            -DOPENCV_GENERATE_PKGCONFIG=ON
            -DWITH_GTK=OFF
            -DWITH_QT=OFF
            -DWITH_OPENGL=OFF
            -DWITH_IPP=OFF
            -DWITH_ITT=OFF
            -DWITH_TBB=OFF
            -DWITH_OPENCL=OFF
    )
    set(OpenCV_FOUND TRUE)
    set(OpenCV_INCLUDE_DIRS ${OPENCV_INSTALL_DIR}/include/opencv4)
    set(OpenCV_LIBS
        ${OPENCV_INSTALL_DIR}/lib/libopencv_calib3d.so
        ${OPENCV_INSTALL_DIR}/lib/libopencv_core.so
    )
endif()

if(OpenCV_FOUND)
    if(NOT OpenCV_LIBS AND OpenCV_LIBRARIES)
        set(OpenCV_LIBS ${OpenCV_LIBRARIES})
    endif()
    message(STATUS "OpenCV was found: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenCV was not found. Please make sure it is installed and accessible.")
endif()

add_library(argus_opencv INTERFACE)
target_include_directories(argus_opencv INTERFACE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(argus_opencv INTERFACE ${OpenCV_LIBS})

get_property(ARGUS_EXTERNAL_DEPS GLOBAL PROPERTY ARGUS_EXTERNAL_DEPS_PROP)
if(NOT ARGUS_EXTERNAL_DEPS)
    set(ARGUS_EXTERNAL_DEPS "")
endif()

# インストール先が指定されなかった場合は、${PROJECT_NAME}ディレクトリにインストールする。
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install CACHE PATH "Default install path" FORCE)
endif()
message(STATUS "CMake install prefix is set to: ${CMAKE_INSTALL_PREFIX}")

# compile options
add_compile_options(
    -DEIGEN_MPL2_ONLY
    $<$<NOT:$<CONFIG:Release>>:-DPYBIND11_DETAILED_ERROR_MESSAGES>
    ${OpenMP_CXX_FLAGS}
)
if(WIN32)
    add_compile_options(/utf-8)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_subdirectory(cpp)
add_subdirectory(python)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME})

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

configure_file(${CMAKE_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    @ONLY)

install(FILES
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME})

