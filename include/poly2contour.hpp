#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>


//Size of the longest side
#define MAX_IMAGE 300



std::vector<std::vector<cv::Point> > read_poly_list_2_contour(const std::string& name);

std::vector<cv::Point> read_poly(std::ifstream& fin, std::vector<float>& correction);

std::vector<cv::Vec4i> extract_hierarchy(int number_of_polygons);
