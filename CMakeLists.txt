# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# (Not part of the boilerplate)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DUDPLOG_PRINTF_TO_UDP -DUDPLOG_PRINTF_ALSO_SERIAL")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(main)
