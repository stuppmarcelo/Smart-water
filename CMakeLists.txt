# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(PROJECT_VER "1.3.4")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Smart-water)

set(PARTITION_TABLE_CSV_PATH "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")

set(SPIFFS_IMAGE_FLASH_SIZE "0x0E0000")

spiffs_create_partition_image(spiffs spiffs FLASH_IN_PROJECT)
