aux_source_directory(. SRC_LIST) #find all source files and save in list of a variate.

include_directories("/usr/xenomai/include")
link_directories("/usr/xenomai/lib")

include_directories("${PROJECT_SOURCE_DIR}/include")
link_directories("${PROJECT_SOURCE_DIR}/lib")

#add_executable(${PROJECT_NAME} robot_client.cpp tcpstream.cpp tcpconnector.cpp)
add_executable(master master.cpp )
add_executable(lan9252_IO lan9252_IO.c )

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

#target_link_libraries(master static libethercat.a)
target_link_libraries(master native rtdm xenomai ethercat_rtdm)
target_link_libraries(lan9252_IO native rtdm xenomai ethercat_rtdm)

