#pragma once
#include <iostream>
#include <string>
#include <memory>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace cvo {

  class DatasetHandler {
  public:
    virtual int read_next_stereo(cv::Mat & left,
                         cv::Mat & right) = 0;
    
    virtual int read_next_lidar_mono(cv::Mat & image,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr pc  ) = 0;
    
    virtual void set_start_index(int start) = 0;
    virtual int get_current_index() = 0;
    virtual int get_total_number() = 0;
    
  };
  
}


