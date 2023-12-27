# CMakeList.txt : CMake project for enerMesh, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

project ("stlReader")

if (UNIX)
    list(APPEND CMAKE_CXX_FLAGS "-pthread -fPIC -Wno-deprecated-enum-enum-conversion")

    include_directories(
    include 
    "/usr/local/include/eigen3/"
    "../triMesh/include" 
    )
else (UNIX)
    if (POLICY CMP0141)
      cmake_policy(SET CMP0141 NEW)
      set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
    endif()

    include_directories(
    include 
    "../../ThirdParty/eigen"
    "../triMesh/include" 
    )
endif(UNIX)

set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/x64/${CMAKE_BUILD_TYPE}")

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
