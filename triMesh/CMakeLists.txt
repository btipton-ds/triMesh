# CMakeList.txt : CMake project for triMesh_2022, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("triMesh")

# Add source to this project's executable.
if (UNIX)
	list(APPEND CMAKE_CXX_FLAGS "-mcmodel=large")

    include_directories(
    include 
    "/usr/local/include/eigen3/"
    "../../utilities/include"
    "../triMesh/include" 
    )
else (UNIX)
    include_directories(
    include 
    "../../ThirdParty/eigen"
    "../../utilities/include"
    "../triMesh/include" 
    )
endif(UNIX)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")

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