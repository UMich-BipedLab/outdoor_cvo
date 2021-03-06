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
#include "utils/Calibration.hpp"
#include "utils/CvoPointCloud.hpp"
#include "cvo/CvoGPU.hpp"
#include "cvo/Cvo.hpp"
using namespace std;
using namespace boost::filesystem;


int main(int argc, char *argv[]) {
  // list all files in current directory.
  //You could put any file path in here, e.g. "/home/me/mwah" to list that directory
  cvo::KittiHandler kitti(argv[1], 0);
  int total_iters = kitti.get_total_number();
  string cvo_param_file(argv[2]);
  string calib_file;
  calib_file = string(argv[1] ) +"/cvo_calib.txt"; 
  cvo::Calibration calib(calib_file);

  std::ofstream outfile(argv[3]);

  int is_geometry_kernel = std::stoi(argv[4]);
  
  cvo::CvoGPU cvo_align(cvo_param_file );
  cvo::CvoParams & init_param = cvo_align.get_params();

  std::vector<std::shared_ptr<cvo::CvoPointCloud>> pcds(total_iters);

  for (int i = 0; i<total_iters; i++) {
    
    // calculate initial guess
    std::cout<<"\n\n\n\n============================================="<<std::endl;
    std::cout<<"Reading "<<i<<std::endl;
    kitti.next_frame_index();
    cv::Mat left, right;
    vector<float> semantics_target;
    if (kitti.read_next_stereo(left, right, NUM_CLASSES, semantics_target) != 0) {
      std::cout<<"finish all files\n";
      break;
    }

    std::shared_ptr<cvo::RawImage> target_raw(new cvo::RawImage(left));
    std::shared_ptr<cvo::CvoPointCloud> target(new cvo::CvoPointCloud(*target_raw, right, calib, false));
    pcds[i] = target;
    
  }


  std::cout<<"Computing self ip\n";
  std::vector<float > self_ip(total_iters);
  for (int i = 0; i < total_iters; i++) {
    Eigen::Matrix4f id;
    id << 1 , 0,0,0, 0,1,0,0,0,0,1,0,0,0,0,1;
    self_ip[i] = std::sqrt(cvo_align.inner_product_gpu(*pcds[i], *pcds[i], id, is_geometry_kernel));
  }
  std::cout<<"Fin computing self ip\n";
  
  for (int i = 0; i < total_iters; i++) {
    for (int j = 0; j < total_iters; j++) {
      if (i == j) continue;
      Eigen::Matrix4f id;
      id << 1 , 0,0,0, 0,1,0,0,0,0,1,0,0,0,0,1;

      float result_ij = cvo_align.inner_product_gpu(*pcds[i], *pcds[j], id, is_geometry_kernel);
      result_ij = result_ij / self_ip[i] / self_ip[j];
      std::cout<<"ip between "<<i<<" and "<<j<<" is "<<result_ij<<std::endl;
      outfile << i<< " "<< j << " "<<result_ij<<"\n";
      
    }
  }

  outfile.close();
  return 0;
}
