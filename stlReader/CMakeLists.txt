# CMakeList.txt : CMake project for enerMesh, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

if (UNIX)
    list(APPEND CMAKE_CXX_FLAGS "-pthread -fPIC")
endif(UNIX)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/${CMAKE_BUILD_TYPE}")

include_directories(
include 
"/usr/local/include/eigen3/"
"../triMesh/include" 
)

link_directories (
"${GLFW_LIB_DIR}"
"${FREE_TYPE_LIB_DER}"
"../${CMAKE_BUILD_TYPE}/"
)

# Add source to this project's executable.
add_library (stlReader 
"src/readStl.cpp"
)

# TODO: Add tests and install targets if needed.
