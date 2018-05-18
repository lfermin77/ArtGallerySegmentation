#include "visibility_graph.hpp"



//Utility Functions
bool detect_concave_triplet(cv::Point p_0, cv::Point p_1, cv::Point p_2){
	int a=1;
	cv::Point v10=p_1-p_0;
	cv::Point v21=p_2-p_1;
	
	float cross_prod = v10.cross(v21);
	
	if( cross_prod <=  0){
		return false;
	}
	else{
		return true;
	}
}







//Visibility Graph Functions
	//Public Functions

Visibility_Graph::Visibility_Graph(){
	decomposed=false;
}

	
std::vector<cv::Point> Visibility_Graph::read_concave_points(){
	if (decomposed){
		std::vector<cv::Point> concave_points;
		for(int i=0;i < concave_points_indices.size(); i++){
			concave_points.push_back(external_contour[concave_points_indices[i]]);
		}
		return concave_points;
	}
	else{
		printf("Please decompose first\n");
	}
}

std::vector< std::pair<cv::Point, cv::Point> > Visibility_Graph::extract_Lines(){
	std::vector< std::pair<cv::Point, cv::Point> > Lines;

	int index=5;
	std::vector<int> visible_to_index = indices_of_visible(index);
	
	for(int i=0;i < visible_to_index.size();i++){
		std::pair<cv::Point, cv::Point> current_line(external_contour[index], external_contour[visible_to_index[i]] );
		Lines.push_back(current_line);
	}
	

	return Lines;
}




	// Private Functions


void Visibility_Graph::write_contour(std::vector<std::vector<cv::Point> > contour_in){
	vector_of_contours = contour_in;
	external_contour = contour_in[0];
	int number_of_points = contour_in[0].size();
	
	for(int i=1; i< contour_in.size(); i++){
		if(contour_in[i].size() > 2){
			set_of_holes.push_back(contour_in[i]);
			number_of_points += contour_in[i].size();
		}
	}
	Oclusion_Adjacency = cv::Mat::zeros(number_of_points, number_of_points, CV_16SC1);
	decomposed=false;
}


void Visibility_Graph::detect_convave_points(){
	concave_points_indices.clear();
		
	if(detect_concave_triplet( external_contour[external_contour.size()-1], external_contour[0], external_contour[1])  ){
		concave_points_indices.push_back(0);
	}	
	for(int i=1; i< (external_contour.size()-1);i++ ){
		if(detect_concave_triplet( external_contour[i-1], external_contour[i], external_contour[i+1])  ){
			concave_points_indices.push_back(i);
		}
	}
	if(detect_concave_triplet( external_contour[external_contour.size()-2], external_contour[external_contour.size()-1], external_contour[0])  ){
		concave_points_indices.push_back(external_contour.size()-1);
	}
}



std::vector<int> Visibility_Graph::indices_of_visible(int index_in){
	std::vector<int> index_visible;
	
	
	int counter_up=index_in + 1;
	if (counter_up < external_contour.size()){
		int last_visible_index = counter_up;
		index_visible.push_back(counter_up);
		while(counter_up < external_contour.size()){
			
			if (detect_concave_triplet(external_contour[index_in], external_contour[last_visible_index], external_contour[counter_up])  ){
				Oclusion_Adjacency.at<int>(cv::Point(index_in,counter_up))=1;
				Oclusion_Adjacency.at<int>(cv::Point(counter_up, index_in))=1;
			}
			else{
				index_visible.push_back(counter_up);
				last_visible_index=counter_up;
			}
			
			counter_up++;
		}
	}
	
	
	return index_visible;
		
}





























void Visibility_Graph::decompose(){
	if (decomposed){
		printf("Already decomposed\n");
	}
	else{
		detect_convave_points();
		
		
		
		
		
	}
	decomposed=true;
}
