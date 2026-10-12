#设置C++编译器
# 使用CMake默认的编译器检测机制，而不是硬编码路径
# set(CMAKE_CXX_COMPILER "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64")
# set(CMAKE_C_COMPILER "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.35.32215\bin\Hostx64\x64")

#添加头文件搜索路径
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/example/cpp/include")
#include_directories("D:\opencv\opencv\sources\include")
#include_directories("D:\opencv\opencv\build\x64\vc15\lib")

# 添加库文件搜索路径
link_directories("${CMAKE_CURRENT_SOURCE_DIR}/lib")
#link_directories("D:\opencv\opencv\build\x64\vc15\lib")
#link_directories("D:\opencv\opencv\build\x64\vc14\lib")


cmake_minimum_required (VERSION 3.12)

#------------------------------------------------------------------------------
# Project setup
project (librokae VERSION 0.2.8)

message(STATUS "")
message(STATUS "    == ${PROJECT_NAME} project configuration ==")
message(STATUS "")

#------------------------------------------------------------------------------
# General settings

# default build type: release
if(NOT CMAKE_BUILD_TYPE )
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "The type of build, options are: Debug, Release, RelWithDebInfo, MinSizeRel."  FORCE )
endif()

# Compiler options
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Be nice to visual studio
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#-------------------------------------------------------------------------------
# Compile option: whether using xMateModel library. Only Linux system supports
option(XCORE_USE_XMATE_MODEL "xMate model library for calculation" OFF)

#-------------------------------------------------------------------------------
# Set default install location to dist folder in build dir
if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/dist" CACHE PATH
        "Install path prefix, prepended onto install directories." FORCE )
endif()

#------------------------------------------------------------------------------
# Included CMakeLists.txt

# External resources/repositories are downloaded here
add_subdirectory(external)

# Public headers
add_subdirectory(example/cpp/include)

# rokae libraries
add_subdirectory(lib)

# Example usages
add_subdirectory(example)

# doc
add_subdirectory(doc)

#-------------------------------------------------------------------------------
# Wrap up of settings printed on build
message(STATUS "")
message(STATUS "    == Final overview for ${PROJECT_NAME} ==")
message(STATUS "Version:              ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH} ${VERSION_TYPE} @ ${VERSION_HOST}")
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "Compiler:             ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_BUILD_TYPE:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  possible options: Debug Release RelWithDebInfo MinSizeRel")
message(STATUS "  set with ` cmake -DCMAKE_BUILD_TYPE=Debug .. `")
message(STATUS "")


