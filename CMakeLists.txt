cmake_minimum_required(VERSION 2.8)
project(forwardKinematics)

# Set the C99 standard for the C files
set(CMAKE_INSTALL_PREFIX /usr)
set(CMAKE_CXX_FLAGS "-g -std=gnu++0x")

# Guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds are not allowed. You may need to remove CMakeCache.txt.")
endif()

# Include Dart and Grip stuff
include_directories(/usr/include/grip)
include_directories(/usr/include/dart)
link_directories(/usr/lib)
set (wxWidgets_USE_LIBS base core gl)
find_package (wxWidgets)
include (${wxWidgets_USE_FILE})

# Link to somatic, amino and ach
# NOTE: Ideally we would like to 'find' these packages but for now, we assume they are either 
# in /usr/lib or /usr/local/lib

# Include Eigen
include_directories(/usr/local/include/eigen3)
include_directories(/usr/include/eigen3)

# Include OpenGL, GLUT and GLU
find_package (OpenGL REQUIRED)
find_package (GLUT REQUIRED)
find_package (GLU REQUIRED)
include_directories(${GLUT_INCLUDE_DIR} ${GLU_INCLUDE_DIR})
link_libraries(${GLU_LIBRARY} "/usr/lib/x86_64-linux-gnu/libglut.so")

# Include kore
include_directories(/usr/include/kore)
link_libraries(kore)

# Include the project files and the common helpers for all the experiments
include_directories(src)
include_directories(src/factors)

# Include gtsam
find_path(gtsam_path "gtsam/base/Lie.h" PATHS "${CMAKE_INSTALL_PREFIX}/include")
find_library(gtsam_lib "gtsam" PATHS "${CMAKE_INSTALL_PREFIX}/lib")
include_directories(AFTER "${gtsam_path}")

# Collect the source, script and fortran files
file(GLOB main_source "src/*.cpp")
LIST(SORT main_source)
file(GLOB planning_source "src/factors/*.cpp")
LIST(SORT planning_source)
file(GLOB scripts_source "exe/*.cpp")
LIST(SORT scripts_source)
file(GLOB tests_source "tests/*.cpp")
LIST(SORT tests_source)

# Build the library and set the executables that use it (have to hardcode for now)
add_library(Main SHARED ${main_source})
add_library(Planning SHARED ${planning_source})

# Add the main executable
add_executable(main exe/main.cpp)
target_link_libraries(main Main) 
target_link_libraries(main protobuf-c amino ach kore somatic filter stdc++ ncurses blas lapack)
target_link_libraries(main ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 

# Add the programming executable
add_executable(planning exe/planning.cpp)
target_link_libraries(planning Planning) 
target_link_libraries(planning "${gtsam_lib}" grip dart boost_system fcl assimp tinyxml tinyxml2 ${wxWidgets_LIBRARIES})

# Build tests 
message(STATUS "\n-- TESTS: ")
foreach(test_src_file ${tests_source})
	get_filename_component(test_base ${test_src_file} NAME_WE)
	message(STATUS "Adding test ${test_src_file} with base name ${test_base}" )
	add_executable(${test_base} ${test_src_file})
	target_link_libraries(${test_base} Main) 
	target_link_libraries(${test_base} ${GRIP_LIBRARIES} ${DART_LIBRARIES} ${DARTExt_LIBRARIES} ${wxWidgets_LIBRARIES}) 
	add_custom_target(${test_base}.run ${test_base} ${ARGN})
endforeach(test_src_file)
message(STATUS " ")
