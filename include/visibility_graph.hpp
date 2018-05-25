#include <stdio.h>
#include <fstream>

#include <math.h>
#include <complex>

#include <opencv2/opencv.hpp>



class Visibility_Graph{
	
	public:
		Visibility_Graph();
		void write_contour(std::vector<std::vector<cv::Point> > contour_in);
		void make_clockwise();
		void decompose();
		std::vector<cv::Point> read_concave_points();
		
		std::vector< std::pair<cv::Point, cv::Point> > extract_Lines();
		
		friend std::ostream& operator<<(std::ostream& os, Visibility_Graph vis);


	
	private:
		bool decomposed;
		std::vector<cv::Point> external_contour;
		std::vector<std::complex<float> > external_complex;
		
		std::vector<std::vector<cv::Point> > set_of_holes;
		std::vector < std::vector<std::complex<float> > > hole_set_complex;

		std::vector<int> concave_points_indices;
		cv::Mat Oclusion_Adjacency;
		int number_of_points;

		


		void detect_convave_points();
		std::vector<int> indices_of_visible(int index_in);
		std::vector< std::vector<int> > simple_visibility();
		void second_visibiliy();
		std::vector< std::pair<int,int> > check_visibility_through_concave_vertex(int index);
		std::vector<int> visible_indices_polar(int index_in);
		bool is_visible (int reference_index, int index, std::set<int> visible_lines_start);
		bool is_visible_point(int index_0, int index_1, int index_start, int index_next);
};
