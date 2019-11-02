#pragma once
#include <string>
#include <vector>
#include <fstream>
#include "DataHandler.hpp"

namespace cvo{

  class KittiHandler : public DatasetHandler {
  public:
    KittiHandler(std::string kitti_folder);
    int read_next_stereo(cv::Mat & left,
                         cv::Mat & right);
    int read_next_stereo(cv::Mat & left,
                         cv::Mat & right,
                         int num_semantic_class,
                         std::vector<float> & left_semantics);
    
   
    int read_next_lidar_mono(cv::Mat & image,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr pc  );
    /*
    int read_next_lidar_mono(cv::Mat & image,
                             std::vector<float> & semantics,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr pc  );
    */
    void set_start_index(int start);
    int get_current_index();
    int get_total_number();
  private:

    int curr_index;
    std::vector<std::string> names;
    std::string folder_name;
    std::ifstream infile;

    bool debug_plot;
  };
  
}