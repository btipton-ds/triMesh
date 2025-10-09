# CMakeList.txt : CMake project for triMesh_2022, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
endif()

project ("triMesh")

add_compile_definitions(_USE_MATH_DEFINES)

# Add source to this project's executable.
if (WIN32)
	message(STATUS "Building Win")
    include_directories(
    "include"
    "../../ThirdParty/eigen"
    "../../utilities/threads/include"
    "../triMesh/include" 
    "../stlReader/include" 
    )

    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/win/${CMAKE_BUILD_TYPE}")
else (WIN32)
	message(STATUS "Building Linux")
	list(APPEND CMAKE_CXX_FLAGS "-mcmodel=large")

    include_directories(
    "include" 
    "/usr/include/eigen3/"
    "../../utilities/threads/include"
    "../stlReader/include"

    )
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
endif(WIN32)

add_library (
    triMesh 
    "src/tm_math.cpp" 
    "src/tm_polyLine.cpp" 
    "src/triMesh.cpp" 
    "src/triMeshPatch.cpp"
    "src/tm_vertex.cpp" 
    "src/tm_vector3.cpp"
    "src/tm_fixedMath.cpp"
    "src/tm_edge.cpp"
    "src/tm_plane.cpp"
    "src/tm_plane_byref.cpp"
    "src/tm_lineSegment.cpp"
    "src/tm_boundingBox.cpp" 
    "src/tm_spatialSearch.cpp"
    
    "../stlReader/src/readWriteStl.cpp"
    "../../utilities/threads/src/MultiCoreUtil.cpp"
  "src/tm_ioUtil.cpp")

link_directories(
../${CMAKE_BUILD_TYPE}
)

if(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
target_compile_features(triMesh PRIVATE cxx_std_20)
endif(WIN32)

message(STATUS "CMAKE_COLOR_DIAGNOSTICS: ${CMAKE_COLOR_DIAGNOSTICS}")