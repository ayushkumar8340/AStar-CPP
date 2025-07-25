#ifndef POST_PROCESSING_H
#define POST_PROCESSING_H

#include "data_types.h"

class PostProcessing
{
    public:
        bool setup(const std::string& param_file);
        bool process(const std::vector<PointData>& input);
        bool process(const std::vector<RealPoint>& input);

        void getOutput(std::vector<RealPoint>& output);
        void getOutput(std::vector<PointData>& output);
        
    private:
        std::vector<RealPoint> real_output_;
        std::vector<PointData> vis_output_;
        double resolution_per_segment_;

        // Map 
        double height_;
        double width_;
        double resolution_;
        double origin_pos_[3];
        double origin_orient_[4];

};  


#endif