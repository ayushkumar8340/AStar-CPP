#ifndef DATA_TYPES_H
#define DATA_TYPES_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <memory>

// Path pixel co-ordinates
typedef struct PointData
{

    int x;
    int y;

    PointData(int x_t = 0,int y_t = 0)
    {
        x = x_t;
        y = y_t;
    }

    PointData(const PointData& new_pt)
    {
        x = new_pt.x;
        y = new_pt.y;
    }

    void copy(int x_t = 0,int y_t = 0)
    {
        x = x_t;
        y = y_t;
    }

    void copy(const PointData& new_pt)
    {
        x = new_pt.x;
        y = new_pt.y;
    }
    
    bool operator==(const PointData& other) const
    {
        return x == other.x && y == other.y;
    }   
    
}PointData;

// Real world co-ordinates for real robot
typedef struct RealPoint
{
    double x,y;
    double theta;

    RealPoint()
    {
        
    }

    RealPoint(const double& x_t,const double& y_t,const double& t_t)
    {
        x = x_t;
        y = y_t;
        theta = t_t;
    }

    RealPoint(const RealPoint& new_pt)
    {
        x = new_pt.x;
        y = new_pt.y;
        theta = new_pt.theta;
    }

    void copy(const RealPoint& new_pt)
    {
        x = new_pt.x;
        y = new_pt.y;
        theta = new_pt.theta;
    }

}RealPoint;

// Hash operator for PointData storage
namespace std {
    template<>
    struct hash<PointData> {
        size_t operator()(const PointData& p) const {
            return hash<int>()(p.x * 10000 + p.y);
        }
    };
}

// custom map data structure
typedef struct MapData
{
    std::vector<uint8_t> data;
    int height;
    int width;
    double resolution;
    double origin_pos[3];
    double origin_orient[4];

    void copy(const MapData& map)
    {
        data.assign(map.data.begin(),map.data.end());
        height = map.height;
        width = map.width;
        resolution = map.resolution;
        origin_pos[0] = map.origin_pos[0];
        origin_pos[1] = map.origin_pos[1];
        origin_pos[2] = map.origin_pos[2];
        origin_orient[0] = map.origin_orient[0];
        origin_orient[1] = map.origin_orient[1];
        origin_orient[2] = map.origin_orient[2];
        origin_orient[3] = map.origin_orient[3];
    }
    
    void reset()
    {
        data.clear();
        height = 0;
        width = 0;
        origin_pos[0] = 0;
        origin_pos[1] = 0;
        origin_pos[2] = 0;

        origin_orient[0] = 0;
        origin_orient[1] = 0;
        origin_orient[2] = 0;
        origin_orient[3] = 1;
        resolution = 0.0;
    }

}MapData;


// Astar planner params
typedef struct 
{
    double inflation_radius;
    double step_size;
    double goal_trans_radius;
    std::string map;
    double diagonal_cost;
    double straight_cost;
    double hue_scaling_factor;
    double vechile_radius;

    void reset()
    {
        inflation_radius = 0;
        step_size = 0;
        goal_trans_radius = 0;
        diagonal_cost = 0;
        straight_cost = 0;
        hue_scaling_factor = 0;
        vechile_radius = 0;
    }

}PlannerParams;

#define IDx(w,i,j) (((w * j) + i))
#define X_IDx(index,width) (index % width)
#define Y_IDx(index,width) (index / width)

#endif