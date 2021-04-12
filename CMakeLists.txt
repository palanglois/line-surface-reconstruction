cmake_minimum_required(VERSION 3.2)
project(line_based_recons_refacto)

option(WITH_OPENMP "Enable parallelism (useful if there are at least 8 cores available)" ON)

### CMAKE FLAGS ###

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON )
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -DCGAL_DISABLE_ROUNDING_MATH_CHECK=ON")

#Adding reference to the media directory
add_definitions(-DTEST_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/")

### EXTERNAL LIBRARIES  ###

# Plane arrangement
link_directories(external/PolyhedralComplex)

#CGAL
set(CGAL_DIR external/cgal/build)
find_package(CGAL 4.11 REQUIRED)
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "")
include( ${CGAL_USE_FILE} )
list(APPEND RECONS_LIBS ${CGAL_LIBRARIES})

#MOSEK
set(MOSEK_DIRECTORY $ENV{MOSEK_DIR})
include_directories("${MOSEK_DIRECTORY}/8/tools/platform/linux64x86/h")
link_directories("${MOSEK_DIRECTORY}/8/tools/platform/linux64x86/bin")
list(APPEND RECONS_LIBS libmosek64.so)

#OpenMP
if(WITH_OPENMP)
    find_package(OpenMP)
    if (OPENMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
        set(CGAL_ADD_LIB gmpxx)
        add_definitions(-DCGAL_USE_GMPXX)
    endif (OPENMP_FOUND)
endif(WITH_OPENMP)

### PROGRAM LIBRARIES ###

#Include directories
include_directories(${CMAKE_SOURCE_DIR})
include_directories(lib/planeArrangement)
include_directories(external)
include_directories(lib/geometricTypes)
include_directories(lib/WeightComputation)
include_directories(lib/meshMetrics)

#Included external libraries
add_subdirectory(external)

# RANSAC on lines
add_subdirectory(lib/ransacOnLines)

# Geometric Types
add_subdirectory(lib/geometricTypes)

# Plane Arrangement
add_subdirectory(lib/planeArrangement)

# Weight Computation
add_subdirectory(lib/WeightComputation)

# Mesh Metrics
add_subdirectory(lib/meshMetrics)

### MAIN APPLICATIONS ###

# Plane detection from lines
add_executable(ransac_on_lines main_ransac.cpp)
target_link_libraries(ransac_on_lines ransacLib)

# Reconstruction from line and detected planes
add_executable(line_based_recons main.cpp)
target_link_libraries(line_based_recons GeometricTypes PlaneArrangement WeightComputation ${CGAL_ADD_LIB} PlaneArr)

# Reconstruction from line and detected planes with run time parameters management
add_executable(line_based_recons_param main_param.cpp)
target_link_libraries(line_based_recons_param GeometricTypes PlaneArrangement WeightComputation ${CGAL_ADD_LIB} PlaneArr)

# Evaluation metrics
add_executable(mesh_metrics main_eval.cpp)
target_link_libraries(mesh_metrics MeshMetrics ${CGAL_ADD_LIB})

### TESTS ###

# Making tests
enable_testing()
include_directories(test)
add_executable(runUnitTests test/mainTest.cpp test/test_geometric_types.cpp
                                              test/test_plane_arrangement.cpp
                                              test/test_weight_computation.cpp
                                              test/test_ransac.cpp)
target_link_libraries(runUnitTests GTest_in GeometricTypes PlaneArrangement WeightComputation pthread
        ransacLib ${CGAL_ADD_LIB} PlaneArr)
add_test(runUnitTests runUnitTests)
