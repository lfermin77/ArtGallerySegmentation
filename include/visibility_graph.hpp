#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>



class Visibility_Graph{
	
	public:
		Visibility_Graph();
		void write_contour(std::vector<std::vector<cv::Point> > contour_in);
		void decompose();
		std::vector<cv::Point> read_concave_points();
		
		std::vector< std::pair<cv::Point, cv::Point> > extract_Lines();
		
		friend std::ostream& operator<<(std::ostream& os, Visibility_Graph vis);


	
	private:
		bool decomposed;
		std::vector<std::vector<cv::Point> > vector_of_contours;
		std::vector<cv::Point> external_contour;
		std::vector<std::vector<cv::Point> > set_of_holes;
		std::vector<int> concave_points_indices;
		cv::Mat Oclusion_Adjacency;
		
		


		void detect_convave_points();
		std::vector<int> indices_of_visible(int index_in);
};
