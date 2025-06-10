cmake_minimum_required (VERSION 3.16)

# Setting for GEM5 & buildroot
# copy mica src to buildroot build folder
execute_process(COMMAND cp -r $ENV{GIT_ROOT}/applications/mica_server ${CMAKE_CURRENT_SOURCE_DIR})
# execute_process(COMMAND make clean WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mica_server/build)
# execute_process(COMMAND cp -r $ENV{GIT_ROOT}/applications/memcached_kernel/gem5-include ${CMAKE_CURRENT_SOURCE_DIR})
# execute_process(COMMAND cp -r $ENV{GIT_ROOT}/applications/memcached_kernel/gem5-lib ${CMAKE_CURRENT_SOURCE_DIR})

# Execute build process

# Set the working directory to ${CMAKE_CURRENT_SOURCE_DIR}/mica_server/build
execute_process(
    COMMAND bash ../configure_server.sh
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mica_server/build
    RESULT_VARIABLE CONFIGURE_RESULT
)

# Check if configure_server.sh script was successful
if(CONFIGURE_RESULT EQUAL "0")
    message(STATUS "configure_server.sh ran successfully")
else()
    message(FATAL_ERROR "configure_server.sh failed with result: ${CONFIGURE_RESULT}")
endif()

# Run 'make' in the same directory
execute_process(
    COMMAND make
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mica_server/build
    RESULT_VARIABLE MAKE_RESULT
)

# Check if 'make' was successful
if(MAKE_RESULT EQUAL "0")
    message(STATUS "Make completed successfully")
else()
    message(FATAL_ERROR "Make failed with result: ${MAKE_RESULT}")
endif()

# add mica server app to buildroot diskimage
install(CODE "execute_process(COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/mica_server/build/netbench_server $ENV{TARGET_DIR}/usr/bin/netbench_server)")
