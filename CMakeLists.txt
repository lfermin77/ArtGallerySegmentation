cmake_minimum_required(VERSION 2.8.3)
project(Segmentation)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package( OpenCV REQUIRED )

##Build
include_directories(
 ${PROJECT_SOURCE_DIR}/include  
)


add_executable( read_poly src/Read_poly_files.cpp include/poly2contour.cpp)

target_link_libraries( read_poly ${OpenCV_LIBS} )
