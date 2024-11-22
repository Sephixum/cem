cmake_minimum_required(VERSION 3.30)

project(
  CERT
  VERSION 0.0.1
  DESCRIPTION "Constexpr software raytracer"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(cert)
target_include_directories(cert PRIVATE src/)
target_sources(cert PRIVATE src/main.cpp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(
    cert
    PRIVATE -O3 #
            -march=native #
            -funroll-loops #
            -fprefetch-loop-arrays #
            -fstrict-aliasing #
            -fno-omit-frame-pointer #
            -fno-elide-constructors #
            -DNDEBUG #
  )

elseif(MSVC)
  target_compile_options(
    cert
    PRIVATE /O2 #
            /GL #
            /fp:fast #
            /arch:AVX2 #
            /GS- #
            /Zc:strictStrings #
            /DNDEBUG #
  )
  target_link_options(cert PRIVATE /LTCG)
endif()

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER}")
message(STATUS "Compiler Flags: ${CMAKE_CXX_FLAGS}")
