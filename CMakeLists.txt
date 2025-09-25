# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.10)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
endif()

if (WIN32)
	add_compile_definitions("WIN32")
else()
    set (CMAKE_COLOR_DIAGNOSTICS "")
endif()

project ("triMesh")

# Include sub-projects.
add_subdirectory ("stlReader" )
add_subdirectory ("triMesh" )
add_subdirectory ("tests" )

