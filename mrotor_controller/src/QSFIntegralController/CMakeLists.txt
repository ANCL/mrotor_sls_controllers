cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ QSFINTEGRALCONTROLLER)

add_library(QSFIntegralController ${QSFINTEGRALCONTROLLER})

target_include_directories (QSFIntegralController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
