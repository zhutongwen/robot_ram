
aux_source_directory(. SRC_LIST) #find all source files and save in list of a variate.

#INCLUDE_DIRECTORIES("/usr/arm-linux-gnueabi/include")
#LINK_LIBRARIES("/usr/arm-linux-gnueabi/lib/libstdc++.so.6")
#LINK_LIBRARIES("/usr/arm-linux-gnueabi/lib/libc.so.6 ")

#add_executable(${PROJECT_NAME} robot_client.cpp tcpstream.cpp tcpconnector.cpp)
add_executable(tinyxml2_test ${SRC_LIST} )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
