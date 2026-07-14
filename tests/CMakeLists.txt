# CMakeList.txt : CMake project for enerMesh, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
endif()

project ("testMath")
add_compile_definitions(_USE_MATH_DEFINES)

if (WIN32)
	message(WARNING "Building Win")
    include_directories(
    "include" 
    "../../ThirdParty/eigen"
    "../triMesh/include" 
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
    "../triMesh/include" 

    )
    set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
    set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
    set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/linux/${CMAKE_BUILD_TYPE}")
endif(WIN32)

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
add_executable (testMath 
"src/triMeshTest.cxx" 
"src/tests.cpp" 
"src/testMesh.cpp" 
"src/testMath.cpp" 
"src/testBoundingBox2D.cpp"
"src/testBoundingBox.cpp" 
"src/testVector3.cpp"
"src/testVector2.cpp")

target_link_libraries("testMath" 
PUBLIC triMesh
PUBLIC stlReader
)

if(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else(WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20 -Wno-deprecated-enum-enum-conversion")
target_compile_features(DistFieldHexMesh PRIVATE cxx_std_20)
endif(WIN32)

# TODO: Add tests and install targets if needed.
