#include "visibility_graph.hpp"




void first(int b){
	int a;
}

Visibility_Graph::Visibility_Graph(){
	a=1;
}

void Visibility_Graph::write_contour(std::vector<std::vector<cv::Point> > contour_in){
	vector_of_contours = contour_in;
	external_contour = contour_in[0];
	for(int i=1; i<contour_in.size(); i++){
		if(contour_in[i].size() > 2){
			set_of_holes.push_back(contour_in[i]);
		}
	}
}


std::vector<cv::Point> Visibility_Graph::detect_convave_points(){
	return external_contour;
}
