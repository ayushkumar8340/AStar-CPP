#include "post_processing.h"

bool PostProcessing::setup(const std::string& param_file)
{
    YAML::Node params = YAML::LoadFile(param_file);
    output_.clear();

    resolution_per_segment_ = params["resolution_per_segment"].as<double>();
    std::string map_file_name = params["map"].as<std::string>() + ".yaml";

    YAML::Node metadata = YAML::LoadFile(map_file_name);

    height_ = metadata["height"].as<double>();
    width_ = metadata["width"].as<double>();
    resolution_ = metadata["resolution"].as<double>();
    origin_pos_[0] = metadata["origin"]["x"].as<double>();
    origin_pos_[1] = metadata["origin"]["y"].as<double>();
    origin_pos_[2] = 0;
    origin_orient_[0] = 0;
    origin_orient_[1] = 0;
    origin_orient_[2] = 0;
    origin_orient_[3] = 1;


}

bool PostProcessing::process(const std::vector<PointData>& input)
{
     
    std::vector<RealPoint> control_points;
    // fill control points based on input;

    process(control_points);

    return true; 
}

bool PostProcessing::process(const std::vector<RealPoint>& input)
{
    real_output_.clear();
    if(input.size() <= 1)
    {
        output = input;
        return false;
    }

    for(int i = 0; i < input.size() - 1; i++)
    {
        for(int j = 0; j < resolution_per_segment_; j++)
        {
            double t = (double)(j / (double)(resolution_per_segment_));
            RealPoint pt;
            pt.x = input[i].x + t * (input[i+1].x - input[i].x);
            pt.y = input[i].y + t * (input[i+1].y - input[i].y);
            real_output_.push_back(pt);
        }

    }

    // fill vis output as well

    return true;
}

bool PostProcessing::getOutput(std::vector<RealPoint>& output)
{
    if(real_output_.size() == 0)
    {
        return false;
    }

    output.clear();

    output = real_output_;

    return true;
}

bool PostProcessing::getOutput(std::vector<PointData>& output)
{
    if(vis_output_.size() == 0)
    {
        return false;
    }

    output.clear();

    output = vis_output_;

    return true;
}