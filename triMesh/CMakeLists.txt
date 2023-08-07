# CMakeList.txt : CMake project for triMesh, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

if (UNIX)
	list(APPEND CMAKE_CXX_FLAGS "-mcmodel=large")
    include_directories(
    include 
    "/usr/local/include/eigen3/"
    "../triMesh/include" 
    )
else (UNIX)
    include_directories(
    include 
    "../../ThirdParty/eigen"
    "../triMesh/include" 
    )
endif(UNIX)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/${CMAKE_BUILD_TYPE}")

add_library (
triMesh 
"src/tm_math.cpp" 
"src/tm_polyLine.cpp" 
"src/triMesh.cpp" 
"src/tm_vertex.cpp" 
"src/tm_vector3.cpp"
)

link_directories(
../${CMAKE_BUILD_TYPE}
)

# TODO: Add tests and install targets if needed.
