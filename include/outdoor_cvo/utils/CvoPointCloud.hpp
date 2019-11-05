#pragma once
#include <string>
#include <cassert>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "utils/data_type.hpp"
#include "utils/RawImage.hpp"
#include "utils/StaticStereo.hpp"
#include "utils/Calibration.hpp"
namespace cvo {


  class CvoPointCloud{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    //const int pixel_pattern[8][2] = {{0,0}, {-2, 0},{-1,-1}, {-1,1}, {0,2},{0,-2},{1,1},{2,0} };
    const int pixel_pattern[8][2] = {{0,0}, {-1, 0},{-1,-1}, {-1,1}, {0,1},{0,-1},{1,1},{1,0} };
    
    //CvoPointCloud(const cv::Mat & left_image,
    //              const cv::Mat & right_image);
    CvoPointCloud(const RawImage & left_raw_image,
                  const cv::Mat & right_image,
                  const Calibration &calib);
    CvoPointCloud();
    CvoPointCloud(const RawImage & left_image,
                  const cv::Mat & right_image,
                  const Calibration &calib,
                  int num_semantic_class,
                  const cv::Mat & semantic_left_img) {assert(0);}
    ~CvoPointCloud();

    int read_cvo_pointcloud_from_file(const std::string & filename);

    // getters
    int num_points() const {return num_points_;}
    int num_classes() const {return num_classes_;}
    const ArrayVec3f & positions() const {return positions_;}
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & labels() const { return labels_;}
    const Eigen::Matrix<float, Eigen::Dynamic, 5> & features() const {return features_;}

    // for visualization via pcl_viewer
    void write_to_color_pcd(const std::string & name) const;
    void write_to_label_pcd(const std::string & name) const;
    void write_to_txt(const std::string & name) const;
   
  private:
    int num_points_;
    int num_classes_;
    
    ArrayVec3f positions_;  // points position. x,y,z
    Eigen::Matrix<float, Eigen::Dynamic, 5> features_;   // rgb, gradient in [0,1]
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> labels_; // number of points by number of classes


    cv::Vec3f avg_pixel_color_pattern(const cv::Mat & raw, int u, int v, int w);

  };
  // for historical reasons
  typedef CvoPointCloud point_cloud;
}
