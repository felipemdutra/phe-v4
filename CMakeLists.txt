cmake_minimum_required(VERSION 3.16)
project(pheV4 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# Add include folder
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(/usr/include)  # system GLM path

# Find external libraries
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

# If wrapgl is installed system-wide (headers in /usr/include/wrapgl/ and library in /usr/lib)
find_path(WRAPGL_INCLUDE_DIR wrapgl/wrapgl.h)
find_library(WRAPGL_LIBRARY wrapgl)

if(NOT WRAPGL_INCLUDE_DIR OR NOT WRAPGL_LIBRARY)
    message(FATAL_ERROR "wrapgl not found. Please provide WRAPGL_INCLUDE_DIR and WRAPGL_LIBRARY.")
endif()

include_directories(${WRAPGL_INCLUDE_DIR})

# Gather sources recursively from src/
file(GLOB_RECURSE SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    OpenGL::GL
    GLEW::GLEW
    glfw
    ${WRAPGL_LIBRARY}
)

