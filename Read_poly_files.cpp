#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>

//using namespace cv;
typedef std::vector<std::vector<cv::Point> > contours;

void read_poly_list_2_contour(const std::string& name);
std::vector<cv::Point> read_poly(std::ifstream& fin);

#define MAX_IMAGE 300


int main(int argc, char** argv )
{
    if ( argc != 2 )
    {
        printf("usage: Read Poly <Image_Path>\n");
        return -1;
    }

    cv::Mat image;
    cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
  
	read_poly_list_2_contour(argv[1]);

    
//    cv::imshow("Display Image", image);

//    cv::waitKey(0);

    return 0;
}

void read_poly_list_2_contour(const std::string& name){
	
	std::ifstream fin(name.c_str());
	printf("Poly name is %s \n", name.c_str());
	
    //remove header commnets
    do{
        char tmp[1024];
        char c=fin.peek();
        if(isspace(c)) fin.get(c); //eat it
        else if(c=='#') {
            fin.getline(tmp,1024);
        }
        else break;
    }while(true);

    //start reading
	int number_of_polygons;
	fin	>> number_of_polygons;
	printf("Number of polygons is %d \n", number_of_polygons);
	contours contour_vector;
	
	for(int i=0; i<number_of_polygons; i++){	
		contour_vector.push_back(read_poly(fin));
	}
	
	
	//Establish hierarchy
	std::vector<cv::Vec4i> hierarchy; //[Next, Previous, First_Child, Parent]
	hierarchy.reserve(number_of_polygons);
	
	if(number_of_polygons == 1){
		cv::Vec4i only_one = cv::Vec<int,4>(-1,-1,-1,-1);
		hierarchy.push_back( only_one );
	}
	else{  //Assume first is father of every other
		cv::Vec4i father = cv::Vec<int,4>(-1,-1,1,-1);
		hierarchy.push_back( father );
		int previous= -1;
		for(int i=1; i< (number_of_polygons-1); i++){
			cv::Vec4i son = cv::Vec<int,4>(i+1,previous,-1,0);
			previous =i;
			hierarchy.push_back( father );
		}
		cv::Vec4i last_son = cv::Vec<int,4>(-1,previous,-1,0);
		hierarchy.push_back( father );
	}

  	fin.close();
}


std::vector<cv::Point> read_poly(std::ifstream& fin){
	
	int contour_size; std::string str_type;
    fin >> contour_size >> str_type;
	
//	printf("Contour Size is %d\n", contour_size);

    if( str_type.find("out")!=std::string::npos )
        printf("Contour \n");
    else printf("Hole \n");
    
    std::vector<float> x_vector, y_vector;
    std::vector<cv::Point> ordered; 

    x_vector.reserve(contour_size);
    y_vector.reserve(contour_size);
    ordered.reserve(contour_size);
    
    for(int i=0; i < contour_size;i++ ){
        double x,y;
        fin >> x >> y;
        
        x_vector.push_back(x);
        y_vector.push_back(y);
      }
    int id;
    float x_min= *std::min_element(x_vector.begin(), x_vector.end() );
    float y_min= *std::min_element(y_vector.begin(), y_vector.end());

    float x_max= *std::max_element(x_vector.begin(), x_vector.end());
    float y_max= *std::max_element(y_vector.begin(), y_vector.end());
    
    float x_rank = x_max - x_min;
    float y_rank = y_max - y_min;
    float max_rank = std::max(x_rank, y_rank);
    float correction = (MAX_IMAGE) / max_rank;
    
    for(int i=0;i < contour_size;i++ ){
        fin>>id; id=id-1;
        
        float x_corrected = x_vector[id]*correction - x_min;
        float y_corrected = y_vector[id]*correction - y_min;
        
        ordered.push_back(cv::Point((int)x_corrected, (int)y_corrected  ) );
        
        //cv::Point2f current_point(x,y);
        //read.push_back(current_point);
        ////double d=x*x+y*y;
        //printf(" (%f, %f) \n", x,y);
    }
   
   return ordered;
}


