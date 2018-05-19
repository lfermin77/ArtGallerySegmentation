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


std::ostream& operator<<(std::ostream& os, Visibility_Graph vis){
	os << "Connectivity \n";
	cv::Size vis_size = vis.Oclusion_Adjacency.size();
	for(int i=0;i < vis_size.width; i++ ){
		for(int j=0;j < vis_size.height; j++ ){
			os << vis.Oclusion_Adjacency.at<int>(cv::Point(i,j) ) << " ";
		}
		os << "\n";
	}
	os << "\n\n";
//	os << 5%4 << "\n";
//	os << (-1)%4 << "\n";
	return os;
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

	
	//~ second_visibiliy();
	std::vector< std::vector<int> > line_set = simple_visibility();
	printf("Number of lines is %d\n",(int)line_set.size());
	
	for(int i=0; i<line_set.size();i++){
		std::vector<int> connection_vec = line_set[i];
		for(int j=0; j<line_set[i].size();j++){
			std::pair<cv::Point, cv::Point> current_line(external_contour[ concave_points_indices[i]  ], external_contour[ connection_vec[j] ] ) ;
			Lines.push_back(current_line);
		}
	}

	
	//~ for(int i=0; i<external_contour.size();i++){
		//~ for(int j=0; j<i;j++){
			//~ if(Oclusion_Adjacency.at<int>(cv::Point(i,j)) == 1){
				//~ 
				//~ std::pair<cv::Point, cv::Point> current_line(external_contour[ i ], external_contour[ j ] ) ;
				//~ Lines.push_back(current_line);
			//~ }
		//~ }
	//~ }
	
	
	
	
	return Lines;
}


void Visibility_Graph::write_contour(std::vector<std::vector<cv::Point> > contour_in){
	vector_of_contours = contour_in;
	external_contour = contour_in[0];
	int number_of_points = contour_in[0].size();
	
	//~ for(int i=1; i< contour_in.size(); i++){
		//~ if(contour_in[i].size() > 2){
			//~ set_of_holes.push_back(contour_in[i]);
			//~ number_of_points += contour_in[i].size();
		//~ }
	//~ }
	Oclusion_Adjacency = cv::Mat::zeros(number_of_points, number_of_points, CV_32SC1);
	decomposed=false;
}




	// Private Functions


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
	//Check for the possible connections of the concave vertex
	
	std::vector<int> index_visible;

	int previous_index = (index_in==0)								? external_contour.size()-1 : index_in-1;
	int next_index     = (index_in==(external_contour.size()-1))	? 0 : index_in+1;	
	
	index_visible.push_back(previous_index);
	index_visible.push_back(next_index);
	
	for(int i=2; i < (external_contour.size()-1); i++ ){
		int round_index = (index_in + i)%(external_contour.size() );
		
		bool at_right = detect_concave_triplet(external_contour[index_in], external_contour[previous_index], external_contour[round_index]);
		bool at_left  = detect_concave_triplet(external_contour[index_in], external_contour[next_index], 	 external_contour[round_index]);
		
		if (!at_right && at_left){
			Oclusion_Adjacency.at<int>(cv::Point(index_in, round_index))=1;
			Oclusion_Adjacency.at<int>(cv::Point(round_index, index_in))=1;
//			index_visible.push_back(round_index);
		}
		else{
			index_visible.push_back(round_index);
		}
	}
	return index_visible;
		
}


std::vector< std::vector<int> > Visibility_Graph::simple_visibility(){
	//Check for every vertex its possible connections
	std::vector< std::vector<int> >  visibility_of_concaves;
	
//	for(int i=0;i<concave_points_indices.size();i++){
	for(int i=0;i<1;i++){
		std::vector<int> current_indices_of_visible = indices_of_visible( concave_points_indices[i]);
		visibility_of_concaves.push_back( current_indices_of_visible );
		printf("Index of visible is %d connected to\n",(int)concave_points_indices[i]);
		for(int j=0; j< current_indices_of_visible.size(); j++){
			printf(" %d ",(int)current_indices_of_visible[j]);
		}
		printf(" \n ");
	}
	return visibility_of_concaves;	
}


void Visibility_Graph::second_visibiliy(){
	
	std::vector< std::vector<int> > possible_connections = simple_visibility();
	
	for(int i=0; i < 1;i++){
//	for(int i=0; i < concave_points_indices.size();i++){
		int current_concave_index = concave_points_indices[i];
		std::vector<int> current_visible = possible_connections[i];

		
		for(int j=0;j< current_visible.size() ;j++){
			for(int k=0;k< j ;k++){
				bool visible = detect_concave_triplet(external_contour[current_visible[j]], external_contour[current_concave_index], external_contour[ current_visible[k] ]);
				if(visible){
					Oclusion_Adjacency.at<int>(cv::Point(j,k))=1;
					Oclusion_Adjacency.at<int>(cv::Point(k,j))=1;
				}
			}
		}
		
	}
	
}



std::vector< std::vector<int> > Visibility_Graph::check_visibility_through_vertex(int index){
	std::vector< std::vector<int> > pepe;
	return pepe;
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
