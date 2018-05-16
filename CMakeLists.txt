cmake_minimum_required(VERSION 2.8.3)
project(Segmentation)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )

add_executable( read_poly Read_poly_files.cpp )
target_link_libraries( read_poly ${OpenCV_LIBS} )
