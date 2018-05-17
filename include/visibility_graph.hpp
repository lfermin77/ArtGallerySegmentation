#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>


void first(int b);

class Visibility_Graph{
	
	public:
		Visibility_Graph();
		void write_contour(std::vector<std::vector<cv::Point> > contour_in);
		std::vector<cv::Point> detect_convave_points();
	
	private:
		int a;
		std::vector<std::vector<cv::Point> > vector_of_contours;
		std::vector<cv::Point> external_contour;
		std::vector<std::vector<cv::Point> > set_of_holes;
};
