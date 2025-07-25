#ifndef PLANNER_UTILS_H
#define PLANNER_UTILS_H

#include "data_types.h"
#include <queue>

#define COSTMAP_SCALING_FACTOR 5.0
#define MAP_UNOCCUPIED_THRESHOLD 255

namespace PlannerUtils
{
    typedef struct
    {
        int x,y;
        double distance;
    }Cell;

    RealPoint pixel2World(const double& res, const RealPoint& map_origin, const PointData& point);
    PointData world2Pixel(const double& res, const RealPoint& map_origin, const RealPoint& point);
    void inflateMap(const MapData& map, const double& inflation_radius, MapData& inflated_map);
};


#endif