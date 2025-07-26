#include "a_star.h"
#include "post_processing.h"


#define PARAM_FILE_LOC "../params/params.yaml"

using namespace std;

bool init_flag = false;
bool final_flag = false;

int init_x,init_y;
int final_x,final_y;

void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN) 
    {
        init_flag = true;
        std::cout << "Intial Point at: (" << x << ", " << y << ")" << std::endl;
        init_x = x;
        init_y = y;

    }

    if(event == cv::EVENT_RBUTTONDOWN)
    {
        final_flag = true;
        std::cout << "Final Point at: (" << x << ", " << y << ")" << std::endl;
        final_x = x;
        final_y = y;

    }
}

int main()
{
    init_x = 0;
    init_y = 0;
    final_x = 0;
    final_y = 0;

    std::unique_ptr<AStar> astar = std::make_unique<AStar>();
    
    if(!astar->setupPlanner(PARAM_FILE_LOC))
    {
        std::cout<<"failed to load param file"<<std::endl;
        return -1;
    }

    MapData loaded_map;
    cv::Mat img;

    cv::namedWindow("map_window");
    cv::setMouseCallback("map_window", onMouse);

    if(!astar->loadMap(loaded_map,img))
    {
        std::cout<<"Failed to load the map"<<std::endl;
        return -1;
    }

    cv::bitwise_not(img,img);
    cv::resize(img, img, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);

    std::vector<RealPoint> path;
    while(1)
    {
        cv::Mat color;
        cv::cvtColor(img,color,cv::COLOR_GRAY2BGR);

        cv::circle(color,cv::Point(init_x,init_y),2,cv::Scalar(255, 0, 0),-1);
        cv::circle(color,cv::Point(final_x,final_y),2,cv::Scalar(0,0,255),-1);

        if(init_flag && final_flag)
        {
            std::cout<<"before plan"<<std::endl;
            astar->plan(PointData(init_x/2,init_y/2),0,PointData(final_x/2,final_y/2),0);
            std::cout<<"planning: "<<astar->getPlan(path)<<std::endl;
            init_flag = false;
            final_flag = false;
        }

        for(int i =0; i < path.size(); i++)
        {
           cv::circle(color,cv::Point((path[i].x*2),(path[i].y *2)),2,cv::Scalar(0, 255, 0),-1); 
        }

        cv::imshow("map_window",color);
        cv::waitKey(1);

    }

    

    return 0;
}



// NOTE !!!

/**
 * This test script gives us the usecase of Astar planner with and without the post processing module.
 * Just comment/uncomment POST_PROCESSING for activating/deactivating the module
 * OpenCV is used for visualizing the map and path
 * It can be easily integrated with any exsiting module just by addding a_star & post_processing library to your code base.
 */

// NOTE !!!