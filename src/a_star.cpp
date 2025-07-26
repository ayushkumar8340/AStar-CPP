#include "a_star.h"

AStar::AStar()
{
    map_loader_ = std::make_unique<CustomMap>();
}

bool AStar::setupPlanner(const std::string& param_file)
{
    YAML::Node params = YAML::LoadFile(param_file);

    params_.inflation_radius = params["inflation_radius"].as<double>();
    params_.step_size = params["step_size"].as<double>();
    params_.goal_trans_radius = params["goal_translation_radius"].as<double>();
    params_.map = params["map"].as<std::string>();
    params_.diagonal_cost = params["diagonal_cost"].as<double>();
    params_.straight_cost = params["straight_cost"].as<double>();
    params_.hue_scaling_factor = params["heuristics_scaling_factor"].as<double>();
    params_.vechile_radius = params["vechile_radius"].as<double>();
    
    return true;
}

bool AStar::loadMap(MapData& map,cv::Mat& img)
{
    MapData map_loaded;
    map_loader_->loadMap(params_.map);
    MapData tmp_map = map_loader_->getRecentMap();
    PlannerUtils::inflateMap(tmp_map,params_.inflation_radius,map_);
    img = map_loader_->getMapImgFromMat(map_);
    
    map.copy(map_);

    for(int i = 0; i < directions_.size(); i++)
    {
        directions_[i].first *= (int)(params_.step_size / map_.resolution);
        directions_[i].second *= (int)(params_.step_size / map_.resolution);
    }

    return true;
}

bool AStar::setMap(const MapData& map)
{
    if(map.data.size() == 0)
    {
        std::cout<<"[Planner] Invalid Map"<<std::endl;
        return false;
    }


    map_.copy(map);
    
    for(int i = 0; i < directions_.size(); i++)
    {
        directions_[i].first *= (int)(params_.step_size / map_.resolution);
        directions_[i].second *= (int)(params_.step_size / map_.resolution);
    }

    std::cout<<"[AStar] Map set"<<std::endl;

    return true;
}
bool AStar::plan(const PointData& start, const double& start_orient, const PointData& target, const double& target_orient)
{
    goal_ = target;
    open_list_ = std::priority_queue<std::pair<int,PointData>,std::vector<std::pair<int,PointData>>,comparatorFunc>();
    open_map_.clear();
    closed_map_.clear();
    
    std::shared_ptr<Node> start_node = std::make_unique<Node>(start);
    start_node->h = calculateHeuristic_(start,target);
    start_node->f = start_node->h;
    open_list_.emplace(start_node->f,start);
    open_map_[start] = start_node;

    while(!open_list_.empty())
    {   
        PointData current_point = open_list_.top().second;
        open_list_.pop();

        std::shared_ptr<Node> current_node = open_map_[current_point];
        open_map_.erase(current_point);
        closed_map_[current_point] = current_node;

        if(goalChecker_(current_point))
        {
            // build path
            path_.clear();
            while(current_node)
            {
                path_.push_back(RealPoint(current_node->x,current_node->y,0));
                current_node = current_node->parent_node;
            }

            std::reverse(path_.begin(),path_.end());
            return true;
        }

        for(auto& dir : directions_)
        {
            int nx = current_point.x + dir.first;
            int ny = current_point.y + dir.second;

            if(!isPointValid_(PointData(nx,ny)))
            {
                continue;
            }
    
            PointData neighbour(nx,ny);

            
            if(closed_map_.count(neighbour))
            {
                continue;
            }

            int g_cost = current_node->g + (((abs(dir.first) + abs(dir.second)) >= ((params_.step_size * 2) / map_.resolution)) ? params_.diagonal_cost : params_.straight_cost);

            std::shared_ptr<Node> neighbour_node;
            
            if(!open_map_.count(neighbour))
            {
                neighbour_node = std::make_unique<Node>(neighbour);
                neighbour_node->g = g_cost;
                neighbour_node->h = calculateHeuristic_(neighbour,target);
                neighbour_node->f = neighbour_node->g + neighbour_node->h;
                neighbour_node->parent_node = current_node;
                open_map_[neighbour] = neighbour_node;
                open_list_.emplace(neighbour_node->f,neighbour);
            }
            else
            {
                neighbour_node = open_map_[neighbour];
                if(g_cost < neighbour_node->g)
                {
                    neighbour_node->g = g_cost;
                    neighbour_node->f = neighbour_node->g + neighbour_node->h;
                    neighbour_node->parent_node = current_node;
                }
            }

            
        }
    }

    return false;

}

void AStar::resetPlanner()
{
    path_.clear();
    open_map_.clear();
    closed_map_.clear();
}

double AStar::calculateHeuristic_(const PointData& p1, const PointData& p2)
{
    return std::round(params_.hue_scaling_factor * (std::sqrt(std::pow(p1.x - p2.x,2) + std::pow(p1.y - p2.y,2))));
}

bool AStar::isPointValid_(const PointData& p)
{
    if(p.x < 0 || p.y < 0 || p.x >= map_.width || p.y >= map_.height || map_.data[IDx(map_.width,p.x,p.y)] != 0)
    {
        return false;
    }

    int radius_int = params_.vechile_radius / map_.resolution;

    for(int dx = -radius_int; dx < radius_int; dx += (int)(params_.step_size/(2.0 *map_.resolution)))
    {
        for(int dy = -radius_int; dy < radius_int;dy += (int)(params_.step_size/(2.0 * map_.resolution)))
        {
            int nx = p.x + dx;
            int ny = p.y + dy;
            if(nx < 0 || ny < 0 || nx >= map_.width || ny >= map_.height)
            {
                return false;
            }
            double dist = (double)std::sqrt(dx*dx + dy*dy) / map_.resolution;
            
            if(map_.data[IDx(map_.width,nx,ny)] != 0)
            {
                return false;
            }
        }

    }
    
    return true;
}

bool AStar::getPlan(std::vector<RealPoint>& path)
{
    if(path_.size() == 0)
    {
        return false;
    }

    path = path_;

    return true;
}

bool AStar::goalChecker_(PointData current_point)
{
    double p_x = goal_.x + params_.goal_trans_radius /  map_.resolution;
    double n_x = goal_.x - params_.goal_trans_radius /  map_.resolution;

    double p_y = goal_.y + params_.goal_trans_radius /  map_.resolution;
    double n_y = goal_.y - params_.goal_trans_radius /  map_.resolution;

    if((current_point.x > (n_x) && current_point.x < (p_x)) && (current_point.y > (n_y) && current_point.y < (p_y)))
    {
        return true;
    }
    
    return false;
}

