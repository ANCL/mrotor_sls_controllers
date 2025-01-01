cmake_minimum_required(VERSION 3.10)

AUX_SOURCE_DIRECTORY(./ QSFGEOMETRICCONTROLLER)

add_library(QSFGeometricController ${QSFGEOMETRICCONTROLLER})

target_include_directories (QSFGeometricController PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
