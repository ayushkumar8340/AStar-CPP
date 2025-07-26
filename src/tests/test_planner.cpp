#include "a_star.h"
#include "post_processing.h"

// Param file location (It also contains a sample map for testing)
#define PARAM_FILE_LOC "../params/params.yaml"

// Comment/Uncomment this for disabling/enabling the post-processing module
// #define POST_PROCESSING

using namespace std;

bool init_flag = false;
bool final_flag = false;

int init_x,init_y;
int final_x,final_y;

// For taking starting and end points (Left mouse button for start point and right for end point)
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

    // Pointer init
    std::unique_ptr<AStar> astar = std::make_unique<AStar>();
    std::unique_ptr<PostProcessing> post = std::make_unique<PostProcessing>();
    
    // Step planner
    if(!astar->setupPlanner(PARAM_FILE_LOC))
    {
        std::cout<<"failed to load param file"<<std::endl;
        return -1;
    }

    // setup post processing module
    if(!post->setup(PARAM_FILE_LOC))
    {
        std::cout<<"failed to load param file"<<std::endl;
        return -1;
    }

    MapData loaded_map;
    cv::Mat img;

    // register callback
    cv::namedWindow("map_window");
    cv::setMouseCallback("map_window", onMouse);

    // Load Map
    if(!astar->loadMap(loaded_map,img))
    {
        std::cout<<"Failed to load the map"<<std::endl;
        return -1;
    }

    // Make it good for visualisation
    cv::bitwise_not(img,img);
    cv::resize(img, img, cv::Size(), 2.0, 2.0, cv::INTER_LINEAR);

    std::vector<PointData> path;

    // Main loop
    while(1)
    {
        cv::Mat color;
        cv::cvtColor(img,color,cv::COLOR_GRAY2BGR);

        // Draw starting and end points
        cv::circle(color,cv::Point(init_x,init_y),2,cv::Scalar(255, 0, 0),-1);
        cv::circle(color,cv::Point(final_x,final_y),2,cv::Scalar(0,0,255),-1);

        // Now plan
        if(init_flag && final_flag)
        {
            std::cout<<"before plan"<<std::endl;
            astar->plan(PointData(init_x/2,init_y/2),0,PointData(final_x/2,final_y/2),0);
            std::cout<<"plan: "<<astar->getPlan(path)<<std::endl;
            init_flag = false;
            final_flag = false;

            // Post processing section
#ifdef POST_PROCESSING
            post->process(path);
            path.clear();
            post->getOutput(path);
#endif        
        }


        // Draw the path
        for(int i =0; i < path.size(); i++)
        {
           cv::circle(color,cv::Point((path[i].x*2),(path[i].y *2)),2,cv::Scalar(0, 255, 0),-1); 
        }

        // Display
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