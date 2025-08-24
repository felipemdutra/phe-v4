cmake_minimum_required(VERSION 3.16)
project(pheV4 LANGUAGES CXX)

# ---------- Basic project settings ----------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)

# ---------- Find external libraries ----------
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

# wrapgl (custom lib) - fail early if missing
find_path(WRAPGL_INCLUDE_DIR wrapgl/wrapgl.h)
find_library(WRAPGL_LIBRARY wrapgl)

if(NOT WRAPGL_INCLUDE_DIR OR NOT WRAPGL_LIBRARY)
    message(FATAL_ERROR "wrapgl not found. Please provide WRAPGL_INCLUDE_DIR and WRAPGL_LIBRARY.")
endif()

# ---------- Automatic source discovery (robust) ----------
# CONFIGURE_DEPENDS makes CMake re-run when files are added/removed.
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
     ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Helpful debug output so you can see what CMake found
message(STATUS "Detected source files:")
foreach(_src IN LISTS SOURCES)
    message(STATUS "  ${_src}")
endforeach()

# ---------- Target ----------
add_executable(${PROJECT_NAME} ${SOURCES})

# Include directories (target-scoped, safer than global include_directories)
# Keep project headers first so your include/ wins over system includes if needed.
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${WRAPGL_INCLUDE_DIR}
)

# ---------- Link libraries ----------
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        OpenGL::GL
        GLEW::GLEW
        glfw
        ${WRAPGL_LIBRARY}
)

# ---------- Optional: sane compile flags for GCC/Clang ----------
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

