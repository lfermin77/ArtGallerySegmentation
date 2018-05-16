#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "poly2contour.hpp"



int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: Read Poly <Image_Path>\n");
        return -1;
    }

    cv::Mat image;
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
  
	std::vector<std::vector<cv::Point> > contour_set = read_poly_list_2_contour(argv[1]);
	std::vector<cv::Vec4i> hierarchy_vector = extract_hierarchy(contour_set.size());    
    
    cv::Mat dst = cv::Mat::zeros(MAX_IMAGE+2*MAX_IMAGE/10, MAX_IMAGE+2*MAX_IMAGE/10, CV_8UC3);
    
    int idx = 0;

//    for( ; idx >= 0; idx = hierarchy_vector[idx][0] )
    for(int idx=0; idx< 1; idx++)
    {
        cv::Scalar color( rand()&255, rand()&255, rand()&255 );
        drawContours( dst, contour_set, idx, color, -1, 8, hierarchy_vector );
	}
        
    cv::imshow("Image from Polygon", dst);
    cv::waitKey(0);

    return 0;
}







