# CMakeList.txt : CMake project for enerMesh, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("triMesh")

include_directories(
include 
../../ThirdParty/eigen
"../triMesh/include" 
)

if (UNIX)
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
else (UNIX)
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
endif(UNIX)

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
