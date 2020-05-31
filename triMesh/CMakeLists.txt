# CMakeList.txt : CMake project for enerMesh, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

if (UNIX)
	list(APPEND CMAKE_CXX_FLAGS "-mcmodel=large")
endif (UNIX)

include_directories(
include 
../../ThirdParty/eigen
)

add_library (
triMesh 
"src/tm_math.cpp" 
"src/tm_polyLine.cpp" 
"src/triMesh.cpp" 
"src/tm_vertex.cpp" 
"src/tm_vector3.cpp"
)


# TODO: Add tests and install targets if needed.
