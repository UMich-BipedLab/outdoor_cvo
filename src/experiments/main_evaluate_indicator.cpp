#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <boost/filesystem.hpp>
//#include <opencv2/opencv.hpp>
#include "dataset_handler/KittiHandler.hpp"
#include "graph_optimizer/Frame.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "utils/CvoPointCloud.hpp"
#include "cvo/CvoGPU.hpp"
#include <pcl/io/pcd_io.h>
//#include "cvo/Cvo.hpp"
using namespace std;
using namespace boost::filesystem;


int main(int argc, char *argv[]) {
  std::cout<<"We are evaluating the indicator\n";
  string cvo_param_file(argv[1]);
  std::ofstream output(argv[2]);
  string sequence(argv[3]);

  string calib_file;
  calib_file = "/home/cel/data/kitti/sequences/"+sequence+"/cvo_calib.txt"; 
  cvo::Calibration calib(calib_file);

  cvo::CvoGPU cvo_align(cvo_param_file );
  cvo::CvoParams & init_param = cvo_align.get_params();
  float ell_init = init_param.ell_init;
  float ell_max = init_param.ell_max;

  for (float init_ell=0.1; init_ell<1.6; init_ell+=0.1){

  
    init_param.ell_init = init_ell; // 0.1;//0.51;
    init_param.ell_max = 2.5;//0.75;
    cvo_align.write_params(&init_param);
    
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();  // from source frame to the target frame
    init_guess(2,3)=0.0;
    Eigen::Affine3f init_guess_cpu = Eigen::Affine3f::Identity();
    init_guess_cpu.matrix()(2,3)=0;
    Eigen::Matrix4f accum_mat = Eigen::Matrix4f::Identity();

    // read the original file
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_pc(new pcl::PointCloud<pcl::PointXYZI>);
    string source_filename = "indicator_evaluation/"+sequence+"_0_afterpointselection.pcd";
    // string source_filename = "indicator_evaluation/"+sequence+"_0_rawpointcloud.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZI> (source_filename, *source_pc) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file\n");
      return (-1);
    }
    std::cout<<"source_filename = "<<source_filename<<std::endl;

    // std::ofstream indicator_file("indicator_evaluation/value_history/"+sequence+"_raw_rotate_indicator_history.csv");
    // std::ofstream function_angle_file("indicator_evaluation/value_history/"+sequence+"_raw_rotate_function_angle_history.csv");
    // std::ofstream indicator_file("indicator_evaluation/value_history/"+sequence+"_raw_rotate_indicator_history_initell_"+std::to_string(init_param.ell_init)+".csv");
    // std::ofstream function_angle_file("indicator_evaluation/value_history/"+sequence+"_raw_rotate_function_angle_history_initell_"+std::to_string(init_param.ell_init)+".csv");
    std::ofstream indicator_file("indicator_evaluation/value_history/"+sequence+"_rotate_indicator_history_initell_"+std::to_string(init_param.ell_init)+".csv");
    std::ofstream function_angle_file("indicator_evaluation/value_history/"+sequence+"_rotate_function_angle_history_initell_"+std::to_string(init_param.ell_init)+".csv");
    // std::ofstream indicator_file("indicator_evaluation/value_history/"+sequence+"_x_translate_indicator_history_initell_"+std::to_string(init_param.ell_init)+".csv");
    // std::ofstream function_angle_file("indicator_evaluation/value_history/"+sequence+"_x_translate_function_angle_history_initell_"+std::to_string(init_param.ell_init)+".csv");

    std::shared_ptr<cvo::Frame> source(new cvo::Frame(0, source_pc,
                                                      // semantics_source,
                                                      calib));
    //0.2));
    double total_time = 0;
    int angle = 0;
    int i = 0;
    for (; angle<=360; angle++) {
      i += 1;

      // string target_filename = "indicator_evaluation/raw_rotation/"+sequence+"_"+std::to_string(angle)+"_rawpointcloud.pcd";
      string target_filename = "indicator_evaluation/rotation/"+sequence+"_0_rotate_"+std::to_string(angle)+"_degree.pcd";
      // string target_filename = "indicator_evaluation/raw_rotation/"+sequence+"_0_raw_rotation_"+std::to_string(angle)+"_degree.pcd";
      // string target_filename = "indicator_evaluation/translation/"+sequence+"_0_x_translate_"+std::to_string(angle)+".pcd";
      // string target_filename = "indicator_evaluation/traslation/"+sequence+"_0_y_translate_"+std::to_string(angle)+".pcd";
      // string target_filename = "indicator_evaluation/traslation/"+sequence+"_0_z_translate_"+std::to_string(angle)+".pcd";

      // calculate initial guess
      std::cout<<"\n\n\n\n============================================="<<std::endl;
      std::cout<<"target_filename = "<<target_filename<<std::endl;

      pcl::PointCloud<pcl::PointXYZI>::Ptr target_pc(new pcl::PointCloud<pcl::PointXYZI>);
      if (pcl::io::loadPCDFile<pcl::PointXYZI> (target_filename, *target_pc) == -1) //* load the file
      {
        PCL_ERROR ("Couldn't read file\n");
        continue;
      }

      std::shared_ptr<cvo::Frame> target(new cvo::Frame(i, target_pc, calib));

      auto source_fr = source->points();
      auto target_fr = target->points();

      Eigen::Matrix4f result, init_guess_inv;
      init_guess_inv = init_guess.inverse();
      float indicator_value = cvo_align.indicator_value(source_fr, target_fr, init_guess_inv, result);

      float function_angle_value = cvo_align.function_angle(source_fr, target_fr, init_guess);

      std::cout<<"indicator value = "<<indicator_value<<std::endl;
      std::cout<<"angle value = "<<function_angle_value<<std::endl;

      indicator_file << angle << " " << indicator_value<<"\n"<<std::flush;
      function_angle_file << angle << " " << function_angle_value<<"\n"<<std::flush;
      
      
      std::cout<<"\n\n===========next frame=============\n\n";
    }
    

    // angle = 1;
    // for (; angle<=5; angle++) {
    //   i += 1;
    //   string target_filename = "indicator_evaluation/"+sequence+"_0_y_translate_"+std::to_string(angle)+".pcd";
    //   // calculate initial guess
    //   std::cout<<"\n\n\n\n============================================="<<std::endl;
    //   std::cout<<"target_filename = "<<target_filename<<std::endl;

    //   pcl::PointCloud<pcl::PointXYZI>::Ptr target_pc(new pcl::PointCloud<pcl::PointXYZI>);
    //   // string target_filename = "indicator_evaluation/"+sequence+"_0_rotate_"+std::to_string(angle)+"_degree.pcd";
    //   // string target_filename = "indicator_evaluation/"+sequence+"_0_raw_rotate_"+std::to_string(angle)+"_degree.pcd";
      
    //   if (pcl::io::loadPCDFile<pcl::PointXYZI> (target_filename, *target_pc) == -1) //* load the file
    //   {
    //     PCL_ERROR ("Couldn't read file\n");
    //     continue;
    //   }

    //   std::shared_ptr<cvo::Frame> target(new cvo::Frame(i, target_pc, calib));

    //   auto source_fr = source->points();
    //   auto target_fr = target->points();

    //   Eigen::Matrix4f result, init_guess_inv;
    //   init_guess_inv = init_guess.inverse();
    //   float indicator_value = cvo_align.indicator_value(source_fr, target_fr, init_guess_inv, result);

    //   float function_angle_value = cvo_align.function_angle(source_fr, target_fr);

    //   std::cout<<"indicator value = "<<indicator_value<<std::endl;
    //   std::cout<<"angle value = "<<function_angle_value<<std::endl;

    //   indicator_file << indicator_value<<"\n"<<std::flush;
    //   function_angle_file << function_angle_value<<"\n"<<std::flush;    
      
    //   std::cout<<"\n\n===========next frame=============\n\n";
    // }

    indicator_file.close();
    function_angle_file.close();
  }

  return 0;
}
