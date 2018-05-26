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



void Visibility_Graph::write_contour(std::vector<std::vector<cv::Point> > contour_in){
	external_contour = contour_in[0];
	number_of_points = contour_in[0].size();
	

	for(int i=0;i<external_contour.size();i++){
		std::complex<float> current_point( external_contour[i].x, external_contour[i].y  );
		external_complex.push_back(current_point);
	}

	Oclusion_Adjacency = cv::Mat::zeros(number_of_points, number_of_points, CV_32SC1);
	
	for(int i=1; i< contour_in.size(); i++){
		std::vector<cv::Point> current_contour = contour_in[i];
		if(current_contour.size() > 2){
			set_of_holes.push_back(current_contour);
			
			std::vector<std::complex<float> > hole_complex;			
			for(int j=0;j<current_contour.size();j++){
				std::complex<float> current_point( current_contour[j].x, current_contour[j].y  );
				hole_complex.push_back(current_point);
			}
			hole_set_complex.push_back(hole_complex);
			
			number_of_points += contour_in[i].size();
		}
	}
	decomposed=false;
}


void Visibility_Graph::make_clockwise(){
	//Fix external
	if( cv::contourArea(external_contour, true)< 0){
		std::reverse(external_contour.begin(),external_contour.end()); 
		std::reverse(external_complex.begin(),external_complex.end());
	}
	
	// Fix Holes
	for(int i=0; set_of_holes.size();i++){
		if( cv::contourArea(set_of_holes[i], true)< 0){
			std::reverse(set_of_holes[i].begin(),set_of_holes[i].end()); 
			std::reverse(hole_set_complex[i].begin(),hole_set_complex[i].end()); 
		}
	}
	
}



std::vector< std::pair<cv::Point, cv::Point> > Visibility_Graph::extract_Lines(){
	std::vector< std::pair<cv::Point, cv::Point> > Lines;

	
	//~ second_visibiliy();
	//~ std::vector< std::vector<int> > line_set = simple_visibility();
	//~ printf("Number of lines is %d\n",(int)line_set.size());
	
	//~ for(int i=0; i<line_set.size();i++){
		//~ std::vector<int> connection_vec = line_set[i];
		//~ for(int j=0; j<line_set[i].size();j++){
			//~ std::pair<cv::Point, cv::Point> current_line(external_contour[ concave_points_indices[i]  ], external_contour[ connection_vec[j] ] ) ;
			//~ Lines.push_back(current_line);
		//~ }
	//~ }

	for(int i=0; i< external_contour.size();i++){
//	{int i=2;
//		std::vector<int> visibles_index2 = visible_indices_polar(concave_points_indices[0]);
		std::vector<int> visibles_index2 = visible_indices_polar(i);
		for(int j=0; j < visibles_index2.size(); j++){
			std::pair<cv::Point, cv::Point> current_line(external_contour[ i  ], external_contour[ visibles_index2[j] ] ) ;
			Lines.push_back(current_line);
		}
	}



	//~ std::vector< std::pair<int,int> > visible_indices = check_visibility_through_concave_vertex(0);
	//~ for(int i=0; i<visible_indices.size();i++){
		//~ std::pair<cv::Point, cv::Point> current_line(external_contour[ visible_indices[i].first  ], external_contour[ visible_indices[i].second ] ) ;
		//~ Lines.push_back(current_line);
	//~ }
	
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

	index_visible.push_back(previous_index);
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



std::vector< std::pair<int,int> > Visibility_Graph::check_visibility_through_concave_vertex(int current_concave_index){
	
	int current_index = concave_points_indices[current_concave_index];
	int previous_index = (current_index==0)							    ? external_contour.size()-1 : current_index-1;
	int next_index     = (current_index==(external_contour.size()-1))	? 0 : current_index+1;
			
	std::vector<int> current_visible = indices_of_visible( current_index );

	//~ std::cout << "The visible indices of "<< current_index <<" are ";
	//~ for(int i=0;i < current_visible.size();i++){
			//~ std::cout << " "<< current_visible[i];
	//~ }
	//~ std::cout << std::endl;
		
	std::vector< std::pair<int,int> > ocluded_lines;
	
	for(int i=0;i < current_visible.size();i++){
	//~ for(int i=0;i < 1;i++){
		//~ int index_2_check = next_index;
		int index_2_check = current_visible[i];		
		std::cout << "     the nex index is: "<< index_2_check  << std::endl;
		
		for(int j=i+1;j< current_visible.size() ;j++){


			
	//		bool visible = detect_concave_triplet(external_contour[previous_index], external_contour[current_index], external_contour[ j ]);
			//~ bool visible = detect_concave_triplet(external_contour[j], external_contour[current_index], external_contour[ next_index ]);
			bool visible = detect_concave_triplet(external_contour[current_visible[j] ], external_contour[current_index], external_contour[ index_2_check ]);
			if(visible){
					Oclusion_Adjacency.at<int>(cv::Point(current_visible[j],index_2_check))=1;
					Oclusion_Adjacency.at<int>(cv::Point(index_2_check,current_visible[j]))=1;
					std::cout << "     the pair ocluded is: "<< current_visible[j] <<" , " << index_2_check << std::endl;
					std::pair<int,int> visible_pair(current_visible[j], index_2_check);
					ocluded_lines.push_back(visible_pair);

			}
			else{
				//~ std::pair<int,int> visible_pair(current_visible[j], index_2_check);
				//~ ocluded_lines.push_back(visible_pair);

			}
			//end j for
		}
	}

	
	
	return ocluded_lines;
}




std::vector<int> Visibility_Graph::visible_indices_polar(int index_in){
	std::vector<int> index_visible;

	int previous_index = (index_in==0)								? external_contour.size()-1 : index_in-1;
	int next_index     = (index_in==(external_contour.size()-1))	? 0 : index_in+1;	

	index_visible.push_back(next_index);


	std::vector<std::complex<float> > relative_position;
	std::complex<float> next_vector     = external_complex[    next_index] - external_complex[index_in];
	std::complex<float> previous_vector = external_complex[previous_index] - external_complex[index_in];

	//Find threshold
	float threshold_angle = std::arg(previous_vector/next_vector) ;	
	threshold_angle = (threshold_angle>0) ? threshold_angle: 2*M_PI+threshold_angle;	
	std::cout << "threshold_angle " << threshold_angle << std::endl;	
	
	
	
	// Order the set of angles
	std::vector<float> angles;
	std::multimap<float,int> angle2indices;
	for(int i=0; i < external_complex.size(); i++ ){
		if(i != index_in){
			std::complex<float> current_relative = external_complex[i] - external_complex[index_in];
			current_relative =  current_relative / next_vector;
			float current_angle = std::arg(current_relative);
			
			if (current_angle < 0){
				current_angle = 2*M_PI + current_angle;
			}
				
			
			if( current_angle <= threshold_angle ){
			
				relative_position.push_back( current_relative );
				std::pair<float, int> index_map(current_angle, i);
				
				angle2indices.insert( index_map);

			}
			std::cout <<"index "<< i<< " has unordered angle " << current_angle << std::endl;
		}
	}
	
	
	//Find the set of lines
	std::set<int> visible_lines_start;
	std::set<int> possible_lines;
	for(int j=1; j < (external_complex.size()-1);j++){
		int circular_index = (index_in + j)%external_complex.size();
		possible_lines.insert(circular_index);
	}
	
	std::cout << "the first visible is vertex "<< next_index << " comes from the line starting in " <<  index_in << std::endl;
//	visible_lines_start.insert(index_in);
	visible_lines_start.insert(next_index);

	visible_lines_start=possible_lines;
	
	std::multimap<float,int>::iterator map_iter = angle2indices.begin();
	map_iter ++;
	for( ; map_iter != angle2indices.end(); map_iter ++ ){
		int current_index = map_iter->second;
		std::cout << "ordered angle " << map_iter->first <<" with index "<< map_iter->second  << std::endl;

		int previous_current_index = (current_index==0)   ? external_contour.size()-1 : current_index-1;
		int next_current_index     = (current_index==(external_contour.size()-1)) ? 0 : current_index+1;

		if(is_visible (index_in, current_index, visible_lines_start))
		{
			index_visible.push_back(current_index);
		}
		/*
		std::cout << "The line visibles are ";
		std::vector <int> index_of_line_closed;
		for (std::set<int>::iterator set_iter = visible_lines_start.begin(); set_iter != visible_lines_start.end(); set_iter++){
			int line_begin = *set_iter;
			int prev = (line_begin==0)? (external_contour.size()-1): (line_begin-1);
			int next = (line_begin==(external_contour.size()-1) )? 0: (line_begin+1);
			
			std::cout << "  " << line_begin <<" to "<< next <<std::endl;
			if( ( current_index  == next ) ){
				std::cout << "    The index  " << current_index <<" close the line "<< previous_current_index  << std::endl;
				index_of_line_closed.push_back(previous_current_index);
			}
			if( ( current_index  == prev )  ){
				std::cout << "    The index  " << current_index <<" close the line "<< next_current_index  << std::endl;
				index_of_line_closed.push_back(next_current_index);
			}
		}
		std::cout << std::endl;
		 
		 
		 
		// First case index close a line and open a new one
		if(index_of_line_closed.size() == 1){
			if(index_of_line_closed[0] < current_index){
				visible_lines_start.erase(index_of_line_closed[0]);
				visible_lines_start.insert(current_index);
			}
			else{
				visible_lines_start.erase(index_of_line_closed[0]);
				
				visible_lines_start.insert(previous_current_index);
			}
		}
		
		//Second case, index is new and open two lines!
		if(index_of_line_closed.size() == 0){		
			visible_lines_start.insert(current_index);
			visible_lines_start.insert(previous_current_index);
		}
		
		
		// Third case, index close two lines
		if(index_of_line_closed.size() == 2){		
			visible_lines_start.erase(current_index);
			visible_lines_start.erase(previous_current_index);
		}

		std::cout << "Lines after erasion ";
		for (std::set<int>::iterator set_iter = visible_lines_start.begin(); set_iter != visible_lines_start.end(); set_iter++){
			int line_begin = *set_iter;
			std::cout << " "<< line_begin;
		}
		std::cout <<std::endl;
	//*/
	}

	
//	angle2indices


	//~ for(int i=2; i < (external_contour.size()-1); i++ ){
		//~ int round_index = (index_in + i)%(external_contour.size() );
		//~ 
	//~ }
	
	
	
	
	
	return index_visible;
}



bool Visibility_Graph::is_visible (int reference_index, int index, std::set<int> visible_lines_start){
	std::complex<float> reference2index = external_complex[index] - external_complex[reference_index];
	float angle_reference = std::arg(reference2index);
	
	for(std::set<int>::iterator set_iter = visible_lines_start.begin(); set_iter != visible_lines_start.end();set_iter++){
		
		int index_start = *set_iter;
		int index_next  = (index_start==(external_contour.size()-1))	? 0 : index_start+1;	
		
		std::cout << "Checkin against line from "<< index_start << " to "<< index_next << std::endl;
		bool point_visible = is_visible_point(reference_index, index, index_start, index_next);
		if(!point_visible){
				std::cout << "The vertex from "<< reference_index << " to "<< index << " is ocluded" << std::endl << std::endl;
			return false;
		}
	}
	std::cout << "The vertex from "<< reference_index << " to "<< index << " is visible" << std::endl << std::endl;
	return true;
}



bool Visibility_Graph::is_visible_point(int index_0, int index_1, int index_start, int index_next){

	//~ std::cout << "Checkin vector from "<< index_0 << " to "<< index_1 << " with line from "<< index_start << " to "<< index_next << std::endl;
	//equations are 
	//x = x0*(1-t0) + x1*(t0)
	//y = y0*(1-t0) + y1*(t0)
	float x0 = external_complex[ index_0 ].real();
	float y0 = external_complex[ index_0 ].imag();
	float x1 = external_complex[ index_1 ].real();
	float y1 = external_complex[ index_1 ].imag();

	
	//equations are 
	//x = x2*t2 + x3*(1-t2)
	//y = y2*t2 + y3*(1-t2)
	float x2 = external_complex[ index_start ].real();
	float y2 = external_complex[ index_start ].imag();
	float x3 = external_complex[ index_next ].real();
	float y3 = external_complex[ index_next ].imag();

	//Solve system 
	//a*t0 + b*t2 = c
	//d*t0 + e*t2 = f
	//	t2 = (af - cd)/(ae - bd)
	// 	t0 = (ce - fb)/(ae - bd)
	
	float a= x1-x0;
	float b= x2-x3;
	float c= x2-x0;
	
	float d= y1-y0;
	float e= y2-y3;	
	float f= y2-y0;

	float 	t0 = (c*e - f*b)/(a*e - b*d);
	float	t2 = (a*f - c*d)/(a*e - b*d) ;
	
	//~ std::cout << "t0: " << t0 << ",  t2: "<< t2 <<std::endl;


	if(  (t2>=0) && (t2<= 1) ){
		//~ std::cout << "    intersection inside line" << std::endl;
		if ((t0<=0) || (t0 >= 1)){
			//~ std::cout << "it is visible" << std::endl;
 			return true;
		}
		else{
			//~ std::cout << "it is ocluded" << std::endl;
				std::cout << "t0: " << t0 << ",  t2: "<< t2 <<std::endl;
			return false;
		}
	}
	else{
		//~ std::cout << "    intersection outside line" << std::endl;
		return true;
	}

	
	return true;
}
















void Visibility_Graph::decompose(){
	if (decomposed){
		printf("Already decomposed\n");
	}
	else{
//		detect_convave_points();
		
		
		
		
		
	}
	decomposed=true;
}
