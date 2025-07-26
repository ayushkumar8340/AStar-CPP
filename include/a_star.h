 
#ifndef A_STAR_H
#define A_STAR_H

#include "unordered_map"
#include "queue"
#include "vector"
#include "functional"
#include "memory"
#include "data_types.h"
#include "iostream"
#include "custom_map.h"
#include "utils.h"

class AStar 
{
    public:
        AStar();
        ~AStar(){}
        bool setupPlanner(const std::string& param_file);
        PlannerParams getParsedParams(){return params_;}
        bool loadMap(MapData& map,cv::Mat& img);
        bool setMap(const MapData& map);
        bool plan(const PointData& start, const double& start_orient, const PointData& target, const double& target_orient);
        bool getPlan(std::vector<RealPoint>& path);
        void resetPlanner();
    private:

        struct Node 
        {
            double x, y, theta; 
            double g = 0.0; // cost start to current       
            double h = 0.0; // hueristic cost
            double f = 0.0; // total cost for queue
            std::shared_ptr<Node> parent_node;     
            
            
            Node(PointData pt)
            {
                x = pt.x;
                y = pt.y;
            }
        };
        
        struct comparatorFunc {
            bool operator()(const std::pair<int, PointData>& a, const std::pair<int, PointData>& b) 
            {
                return a.first > b.first;
            }
        };

        double calculateHeuristic_(const PointData& p1, const PointData& p2);
        bool isPointValid_(const PointData& p);
        bool goalChecker_(PointData goal_point);

        std::priority_queue<std::pair<int,PointData>,std::vector<std::pair<int,PointData>>,comparatorFunc> open_list_;
        std::unordered_map<PointData,std::shared_ptr<Node>> open_map_;
        std::unordered_map<PointData,std::shared_ptr<Node>> closed_map_;

        PlannerParams params_;

        MapData map_;

        std::vector<RealPoint> path_;

        PointData goal_;

        std::unique_ptr<CustomMap> map_loader_;

        std::vector<std::pair<int ,int>> directions_ = 
        {
            {-1, -1}, {-1, 0}, {-1, 1},
            { 0, -1},          { 0, 1},
            { 1, -1}, { 1, 0}, { 1, 1}
        };
        

};


#endif