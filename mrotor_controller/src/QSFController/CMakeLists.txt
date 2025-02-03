cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ QSFCONTROLLER)

add_library(QSFController ${QSFCONTROLLER})

target_include_directories (QSFController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
