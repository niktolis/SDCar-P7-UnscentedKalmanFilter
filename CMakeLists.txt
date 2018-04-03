project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CMAKE_BUILD_TYPE Debug)
set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources 
    ./src/ukf.cpp 
    ./src/main.cpp 
    ./src/tools.cpp
    ./src/ukf.h
    ./src/tools.h
    ./src/measurement_package.h)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(UnscentedKF ${sources})

# Visual Studio and MSVC Support for compiling and running on Windows
if(MSVC)
set(CMAKE_TOOLCHAIN_FILE "c:\\vcpkg\\scripts\\buildsystems\\vcpkg.cmake")
add_definitions(-DWIN_UWS_LIB)
else(MSVC)
target_link_libraries(UnscentedKF z ssl uv uWS)
endif(MSVC)
