# g2o 坑列表
---
1. list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

2. # CSparse
	find_package(*CSparse* REQUIRED)
	include_directories(${*CSPARSE*_INCLUDE_DIR})
	target_link_libraries(g2o
    ${CSPARSE_LIBRARY})

3. CMake Warning at cmake_modules/FindQGLViewer.cmake:1 (FIND_PACKAGE):
   By not providing "FindQt5.cmake" in CMAKE_MODULE_PATH
   $ sudo apt-get install cmake qt5-default qtcreator

