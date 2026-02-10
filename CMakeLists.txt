# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(blufi_demo)

#-----------------------My section--------------------------------
#execute_process(
#	COMMAND ctags -R --c++-kinds=+p --fields=+iaS --extras=+q  /opt/esp-idf/
#	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} 
#)
##----------------------Not working))------------------------------
#add_custom_target(tags) 
#add_custom_command(
#    TARGET tags
#    POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E cd ${CMAKE_CURRENT_SOURCE_DIR} # Optional: change directory
#    COMMAND ${CMAKE_COMMAND} -E echo "-------------- Post-build command running ---"
#    COMMAND /bin/bash -c "./ctags.sh ${CMAKE_BINARY_DIR}"      # Launch a shell command/script
#    COMMENT "----------------Running post-build script for MyApp"                 # Message displayed during build
#)
