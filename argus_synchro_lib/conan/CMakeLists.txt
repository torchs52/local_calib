cmake_minimum_required(VERSION 3.24)
project(argus_synchro_lib VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PYBIND11_FINDPYTHON ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

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

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ArgusSynchroConanDeps.cmake)

add_compile_options(
    -DEIGEN_MPL2_ONLY
    $<$<NOT:$<CONFIG:Release>>:-DPYBIND11_DETAILED_ERROR_MESSAGES>
    ${OpenMP_CXX_FLAGS}
)
if(WIN32)
    add_compile_options(/utf-8)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/../install CACHE PATH "Default install path" FORCE)
endif()
message(STATUS "CMake install prefix is set to: ${CMAKE_INSTALL_PREFIX}")

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp cpp)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../python python)

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

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/../cmake/${PROJECT_NAME}Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    @ONLY
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME})
