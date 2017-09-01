# g2o 坑列表
---
1. Only  SlamBookCode/3rdparty/g2o 才能用！！！

2. list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

3. # CSparse
	find_package(*CSparse* REQUIRED)
	include_directories(${*CSPARSE*_INCLUDE_DIR})
	target_link_libraries(g2o
    ${CSPARSE_LIBRARY})

4. CMake Warning at cmake_modules/FindQGLViewer.cmake:1 (FIND_PACKAGE):
   By not providing "FindQt5.cmake" in CMAKE_MODULE_PATH
   $ sudo apt-get install cmake qt5-default qtcreator

