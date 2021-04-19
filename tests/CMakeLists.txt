# CMakeList.txt : CMake project for enerMesh, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

include_directories(
include 
../../ThirdParty/eigen
"../triMesh/include" 
)

# Add source to this project's executable.
add_library (triMeshTest 
"src/triMeshTest.cxx" 
"src/tests.cpp" 
"src/testMesh.cpp" 
"src/testMath.cpp" 
"src/testBoundingBox.cpp" 
"src/testVector3.cpp")

target_link_libraries("triMeshTest" 
PUBLIC triMesh
PUBLIC stlReader
PUBLIC enerMesh
)

# TODO: Add tests and install targets if needed.
