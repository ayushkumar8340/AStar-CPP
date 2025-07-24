#include "custom_map.h"

void CustomMap::setMapMetaData(const MapData& map,const std::string& filename)
{
    map_.copy(map);
    filename_ = filename;
}

void CustomMap::showCurrentMap(const std::string& win_name)
{
    cv::Mat map_image(map_.height,map_.width,CV_8UC1);
    for(int i = 0; i < map_.height; i++)
    {
        for(int j = 0; j < map_.width; j++)
        {
            map_image.at<uchar>(i,j) = static_cast<uchar>(map_.data[IDx(map_.width,j,i)]);
        }
    }
    cv::resize(map_image,map_image,cv::Size(),0.5,0.5);
    cv::imshow(win_name,map_image);
}

cv::Mat CustomMap::getMapImgFromMat(const MapData& map)
{
    cv::Mat map_image(map.height,map.width,CV_8UC1);
    for(int i = 0; i < map.height; i++)
    {
        for(int j = 0; j < map.width; j++)
        {
            map_image.at<uchar>(i,j) = static_cast<uchar>(map.data[IDx(map.width,j,i)]);
        }
    }

    return map_image;
}

bool CustomMap::saveMap()
{
    cv::Mat map_image(map_.height,map_.width,CV_8UC1);
    for(int i = 0; i < map_.height; i++)
    {
        for(int j = 0; j < map_.width; j++)
        {
            map_image.at<uchar>(i,j) = static_cast<uchar>(map_.data[IDx(map_.width,j,i)]);
        }
    }

    std::string img_fname = filename_ + ".png";
    if(!cv::imwrite(img_fname,map_image))
    {
        return false;
    }

    YAML::Node metadata;
    metadata["resolution"] = map_.resolution;
    metadata["height"] = map_.height;
    metadata["width"] = map_.width;
    metadata["origin"]["x"] = map_.origin_pos[0];
    metadata["origin"]["y"] = map_.origin_pos[0];
    metadata["origin"]["yaw"] = 0;

    std::string meta_fname = filename_ + ".yaml";

    std::ofstream out_file(meta_fname);
    if(!out_file.is_open())
    {
        return false;
    }

    out_file << metadata;
    out_file.close();

    return true;
}

bool CustomMap::loadMap(const std::string& file)
{
    std::string meta_fname = file + ".yaml";
    YAML::Node metadata = YAML::LoadFile(meta_fname);
    map_.reset();
    map_.height = metadata["height"].as<double>();
    map_.width = metadata["width"].as<double>();
    map_.resolution = metadata["resolution"].as<double>();
    map_.origin_pos[0] = metadata["origin"]["x"].as<double>();
    map_.origin_pos[1] = metadata["origin"]["y"].as<double>();
    map_.origin_pos[2] = 0;
    map_.origin_orient[0] = 0;
    map_.origin_orient[1] = 0;
    map_.origin_orient[2] = 0;
    map_.origin_orient[3] = 1;

    std::string img_fname = file + ".pgm";
    cv::Mat map_image = cv::imread(img_fname,cv::IMREAD_GRAYSCALE);
    map_image_ = cv::Mat();
    map_image_ = map_image.clone();
    if(map_image.empty() || map_image.rows != map_.height || map_image.cols != map_.width)
    {
        std::cout<<"incorrect image loaded"<<std::endl;
        return false;
    }
    
    map_.data.resize(map_.height * map_.width);

    for(int i = 0; i < map_.height; i++)
    {
        for(int j = 0; j < map_.width; j++)
        {
            map_.data[IDx(map_.width,j,i)] = static_cast<uchar>(map_image.at<uchar>(i,j));
        }
    }

    return true;
}

MapData& CustomMap::getRecentMap()
{
    return map_;
}

bool CustomMap::getRecentMapImage(cv::Mat& img)
{
    if(map_image_.empty())
    {   
        return false;
    }

    img = map_image_;

    return true;
}