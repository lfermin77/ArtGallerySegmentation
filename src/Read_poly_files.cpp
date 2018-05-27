#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "poly2contour.hpp"
#include "visibility_graph.hpp"


int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: Read Poly <Image_Path>\n");
        return -1;
    }

  
	//Transform poly format to contour format
	std::vector<std::vector<cv::Point> > contour_set = read_poly_list_2_contour(argv[1]);
	std::vector<cv::Vec4i> hierarchy_vector = extract_hierarchy(contour_set.size());    
    
    //Reduce contour size
    float epsilon = 5;
    
    std::vector<std::vector<cv::Point> > reduced_contour;
    for(int i=0;i<contour_set.size();i++){
		std::vector<cv::Point> current_reduced_contour;
		approxPolyDP(contour_set[i], current_reduced_contour, epsilon, 1);
		reduced_contour.push_back(current_reduced_contour);
//		printf("Number of points in contour %d, previous: %d, reduced %d\n", i, (int)contour_set[i].size(), (int)current_reduced_contour.size() );
	}

	
	
	//Visibility Graph
    Visibility_Graph vis_graph;
    vis_graph.write_contour(reduced_contour);
    vis_graph.make_clockwise();
    vis_graph.decompose();
//    std::vector<cv::Point> concave_points = vis_graph.read_concave_points();

    
    std::vector< std::pair<cv::Point, cv::Point> > lines = vis_graph.extract_Lines();
    
    std::vector<cv::Point> guard_points = vis_graph.guard_points();
    
    std::cout << vis_graph;

    
    //Draw Image
    cv::Mat dst = cv::Mat::zeros(MAX_IMAGE+2*MAX_IMAGE/10, MAX_IMAGE+2*MAX_IMAGE/10, CV_8UC3);
    
		//Draw Contours
    int idx = 0;
    for( ; idx >= 0; idx = hierarchy_vector[idx][0] ){
//    for(int idx=0; idx< 1; idx++){
        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
//        drawContours( dst, contour_set, idx, color, -1, 8, hierarchy_vector );
        drawContours( dst, reduced_contour, idx, color, -1, 8, hierarchy_vector );
	}
		//Draw Circles
	//~ for(int i=0; i< concave_points.size();i++){
		//~ cv::circle(dst, concave_points[i], 3, cv::Scalar( 0, 0, 255), -1);
	//~ }
	for(int i=0; i< guard_points.size();i++){
		cv::circle(dst, guard_points[i], 3, cv::Scalar( 0, 0, 255), -1);
	}
		//Draw Lines
	//~ for(int i=0; i< lines.size();i++){
		//~ cv::line(dst, lines[i].first, lines[i].second, cv::Scalar( 0, 0, 255), 1);
	//~ }
		//~ 
		
    
    
    //Show Image    
	cv::namedWindow("Image_from_Polygon", cv::WINDOW_AUTOSIZE );
    cv::imshow("Image_from_Polygon", dst);
    cv::waitKey(0);

    return 0;
}







