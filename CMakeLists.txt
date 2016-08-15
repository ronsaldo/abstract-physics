# Leng cmake build script.
cmake_minimum_required(VERSION 2.8)
project(APHY)

if(APPLE)
	set(CMAKE_OSX_ARCHITECTURES "x86_64;i386")
endif()

option(APHY_BUILD_SAMPLES "Build APHY Samples" OFF)

# Check the build type
if (CMAKE_BUILD_TYPE STREQUAL "")
	# CMake defaults to leaving CMAKE_BUILD_TYPE empty. This screws up
	# differentiation between debug and release builds.
	set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None (CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif ()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Output paths.

set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dist")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/;${CMAKE_MODULE_PATH}")

# Turn warnings
if (MSVC)
	# using Visual Studio C++
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else()
	set(CMAKE_C_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c11")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

if(UNIX)
	set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fvisibility=hidden")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fvisibility=hidden")
endif()

# Add some include directories
include_directories(. include)

# Set output dir.
set(EXECUTABLE_OUTPUT_PATH "${APHY_BINARY_DIR}/dist")
set(LIBRARY_OUTPUT_PATH "${APHY_BINARY_DIR}/dist")

# Find unittest++
#pkg_check_modules(UNITTESTMM unittest++)
#if(UNITTESTMM_FOUND)
#    include_directories(${UNITTESTMM_INCLUDE_DIRS})
#endif()

# Find OpenGL
find_package(OpenGL)

if(D3D12_INCLUDE_DIRS AND D3D12_LIBRARY)
	set(D3D12_FOUND TRUE)
endif()

# For VisualStudio in Window
find_path(SDL2_INCLUDE_DIRS
  NAMES SDL.h
  PATH_SUFFIXES SDL2
)

find_path(SDL2_LIBRARY_PATH
  NAMES SDL2.lib SDL2.a SDL2-2.0.lib SDL2-2.0.a libSDL2-2.0.a
)

find_library(SDL2_LIBRARY NAMES SDL2 SDL2-2.0 PATHS ${SDL2_LIBRARY_PATH})
find_library(SDL2_MAIN_LIBRARY NAMES SDL2main PATHS ${SDL2_LIBRARY_PATH})

if(SDL2_INCLUDE_DIRS AND SDL2_LIBRARY AND SDL2_INCLUDE_DIRS)
    # Use the SDL2 include path.
    include_directories(${SDL2_INCLUDE_DIRS})

	set(SDL2_FOUND TRUE)
	if(SDL2_MAIN_LIBRARY)
    	set(SDL2_LIBRARIES ${SDL2_LIBRARY} ${SDL2_MAIN_LIBRARY})
	else()
		set(SDL2_LIBRARIES ${SDL2_LIBRARY})
	endif()
endif()


# Samples libraries
set(APHY_MAIN_LIB APhy)
set(APHY_SAMPLES_LIBS ${APHY_SAMPLES_LIBS} ${SDL2_LIBRARIES} ${APHY_MAIN_LIB})

# Build the implementations
add_subdirectory(implementations)

# Build the samples
if(SDL2_FOUND AND APHY_BUILD_SAMPLES)
	add_subdirectory(samples)
endif()

# Build the tests.
if(UNITTESTMM_FOUND)
    add_subdirectory(tests)
endif()
