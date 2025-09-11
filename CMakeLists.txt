cmake_minimum_required(VERSION 3.15...4.0)

project(refx VERSION 0.1.0
             DESCRIPTION "refx - C++ core library for geographic and navigation computations"
             LANGUAGES CXX)

# Compiler extensions
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wall)
endif()

# Enable/Disable Eigen Detection
set(REFX_ENABLE_EIGEN_DETECTION ON CACHE BOOL "Enable the dection of Eigen3 Library")
 
add_subdirectory(src)


# Testing
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
    add_subdirectory(test)
endif()

