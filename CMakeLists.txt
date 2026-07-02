############################################################################
#                                                                          #
#  file: CMakeLists.txt                                                    #
#  Copyright (C) 2026                                                      #
#      Mattia Piazza                                                       #
#      Department of Industrial Engineering.                               #
#      University of Trento                                                #
#      email: mattia.piazza@unitn.it                                       #
#                                                                          #
############################################################################

cmake_minimum_required(VERSION 3.14)

project(vprim VERSION 0.1.0 LANGUAGES CXX)

# Set modern CMake policies
# CMP0135: Use extraction-time timestamps for FetchContent downloads
# This prevents issues where dependencies might not be rebuilt if the URL changes.
cmake_policy(SET CMP0135 NEW)

# Global settings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find OS Threads (Modern way, handles Windows/Unix automatically)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

#   ___ 
#  / __| ___ _  _ _ _ __ ___ ___ 
#  \__ \/ _ \ || | '_/ _/ -_|_-< 
#  |___/\___/\_,_|_| \__\___/__/ 
#
# CONFIGURE_DEPENDS ensures CMake re-runs automatically if you add a new .cc file
file(GLOB SOURCES CONFIGURE_DEPENDS "src/*.cc")

# Define the static library
add_library(vprim STATIC ${SOURCES})

#   _____                  _ 
#  |_   _|_ _ _ _ __ _ ___| |_ ___ 
#    | |/ _` | '_/ _` / -_)  _(_-< 
#    |_|\__,_|_| \__, \___|\__/__/ 
#                |___/ 
#
# PUBLIC tells CMake: "vprim needs this to compile, AND anyone who links to vprim needs it too"
target_include_directories(vprim PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

# Link necessary system libraries to the target
target_link_libraries(vprim PUBLIC Threads::Threads)
if(WIN32)
    target_link_libraries(vprim PUBLIC ws2_32 iphlpapi kernel32)
endif()

#   _____       _ 
#  |_   _|__ __| |_ ___ 
#    | |/ -_|_-<  _(_-< 
#    |_|\___/__/\__/__/ 
#
enable_testing()
add_subdirectory(src_tests)