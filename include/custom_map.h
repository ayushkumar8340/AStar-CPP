#ifndef CUSTOM_MAP_H
#define CUSTOM_MAP_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include "data_types.h"

class CustomMap
{
    public:
        void setMapMetaData(const MapData& map, const std::string& filename);
        bool saveMap();
        bool loadMap(const std::string& file);
        MapData& getRecentMap();
        bool getRecentMapImage(cv::Mat& map);
        void showCurrentMap(const std::string& win_name);
        cv::Mat getMapImgFromMat(const MapData& map);

    private:
        MapData map_;
        std::string filename_;
        cv::Mat map_image_;
};



#endif