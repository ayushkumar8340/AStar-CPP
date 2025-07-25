#include "utils.h"

RealPoint PlannerUtils::pixel2World(const double& res, const RealPoint& map_origin, const PointData& point)
{
    RealPoint r;
    r.x = (map_origin.x + (point.x * res));
    r.y = (map_origin.y + (point.y * res));
    return r;
}

PointData PlannerUtils::world2Pixel(const double& res,const RealPoint& map_origin,const RealPoint& point)
{
    PointData p;
    p.x = (int)((point.x - map_origin.x)/res);
    p.y = (int)((point.y - map_origin.y)/res);
    return p;
}

void PlannerUtils::inflateMap(const MapData& map, const double& inflation_radius, MapData& inflated_map)
{
    double cost_scaling_factor = COSTMAP_SCALING_FACTOR;
    inflated_map.copy(map);
    int radius = static_cast<int>(std::ceil(inflation_radius/map.resolution));
    std::queue<PlannerUtils::Cell> q;
    std::vector<bool> visited(map.width * map.height,false);

    for(int y = 0; y < map.height; y++)
    {
        for(int x = 0; x < map.width; x++)
        {
            if(map.data[IDx(map.width,x,y)] <= (MAP_UNOCCUPIED_THRESHOLD-10))
            {
                q.push({x,y,0.0});
                visited[IDx(map.width,x,y)] = true;
                inflated_map.data[IDx(map.width,x,y)] = 255;
            }
            else
            {
                inflated_map.data[IDx(map.width,x,y)] = 0;
            }
        }
    }

    const int dx[8] = {1, -1, 0, 0,  1, -1, -1, 1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while(!q.empty())
    {
        Cell c = q.front();
        q.pop();

        int idx =  IDx(map.width,c.x,c.y);
        double cost = 125.0 * std::exp(-cost_scaling_factor * c.distance);
        if(cost < 1.0)
        {
            continue;
        }

        if(cost > inflated_map.data[idx])
        {
            inflated_map.data[idx] = static_cast<uint8_t>(std::round(cost));
        }

        for(int i = 0; i < 8; i++)
        {
            int nx = c.x + dx[i];
            int ny = c.y + dy[i];

            if(nx < 0 || nx >= map.width || ny < 0 || ny >= map.height)
            {
                continue;
            }

            int nidx = IDx(map.width,nx,ny);
            if(visited[nidx])
            {
                continue;
            }

            double new_dist = c.distance + map.resolution;
            if(new_dist > inflation_radius)
            {
                continue;
            }

            q.push({nx,ny,new_dist});
            visited[nidx] = true;
        }
    }

}