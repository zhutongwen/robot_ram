project(ecat_master)
cmake_minimum_required(VERSION 2.8)


##arm-linux
## this is required
#set(CMAKE_SYSTEM_NAME Linux)

## specify the cross compiler
#set(CMAKE_C_COMPILER   "/usr/bin/arm-linux-gnueabi-gcc")
#set(CMAKE_CXX_COMPILER "/usr/bin/arm-linux-gnueabi-g++")

##set(CMAKE_C_COMPILER   "/home/z/work/arm/arm-2009q3/bin/arm-linux-gcc")
##set(CMAKE_CXX_COMPILER "/home/z/work/arm/arm-2009q3/bin/arm-linux-g++")


##SET(CMAKE_FIND_ROOT_PATH "编译器环境路径")
#set(CMAKE_FIND_ROOT_PATH "/usr/arm-linux-gnueabi")
##set(CMAKE_FIND_ROOT_PATH "/home/z/work/arm/arm-2009q3/arm-none-linux-gnueabi")

#set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
#set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
#set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

#linux
#set(CMAKE_C_COMPILER "/usr/bin/gcc")
#set(CMAKE_CXX_COMPILER "/usr/bin/g++")

#set(CMAKE_C_FLAGS " -pthread ")
#set(CMAKE_CXX_FLAGS "-std=c++11 -pthread -static")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")


#add_subdirectory(src/socket)
#add_subdirectory(src/tinyxml2)
add_subdirectory(src/etherlab)
