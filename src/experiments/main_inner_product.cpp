#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <boost/filesystem.hpp>
#include "dataset_handler/KittiHandler.hpp"
#include "utils/RawImage.hpp"
#include "utils/Calibration.hpp"
#include "utils/CvoPointCloud.hpp"
#include "cvo/CvoGPU.hpp"
#include "cvo/CvoParams.hpp"

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
  int start_frame = std::stoi(argv[4]);
  kitti.set_start_index(start_frame);
  int max_num = std::stoi(argv[5]);
  
  
  cvo::CvoGPU cvo_align(cvo_param_file );
  cvo::CvoParams & init_param = cvo_align.get_params();
  float ell_init = init_param.ell_init;
  float ell_decay_rate = init_param.ell_decay_rate;
  int ell_decay_start = init_param.ell_decay_start;
  init_param.ell_init = init_param.ell_init_first_frame;
  init_param.ell_decay_rate = init_param.ell_decay_rate_first_frame;
  init_param.ell_decay_start  = init_param.ell_decay_start_first_frame;
  cvo_align.write_params(&init_param);

  std::cout<<"write ell! ell init is "<<cvo_align.get_params().ell_init<<std::endl;

  //cvo::cvo cvo_align_cpu("/home/rayzhang/outdoor_cvo/cvo_params/cvo_params.txt");

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();  // from source frame to the target frame
  //init_guess(2,3)=2.22;
  Eigen::Matrix4f accum_mat = Eigen::Matrix4f::Identity();

  cv::Mat source_left, source_right;
  kitti.read_next_stereo(source_left, source_right);
  std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left));
  std::shared_ptr<cvo::CvoPointCloud> source(new cvo::CvoPointCloud(*source_raw, source_right, calib, true));
  
  //for (int i = start_frame; i< start_frame + 1 ; i++) {
  for (int i = start_frame; i< min(total_iters, start_frame+max_num)-1; i++) {
    
    // calculate initial guess
    std::cout<<"\n\n\n\n============================================="<<std::endl;
    std::cout<<"Evaluating inner product "<<i<<" and "<<i+1<<" with GPU "<<std::endl;

    kitti.next_frame_index();
    cv::Mat left, right;
    //vector<float> semantics_target;
    //if (kitti.read_next_stereo(left, right, 19, semantics_target) != 0) {
    if (kitti.read_next_stereo(left, right) != 0) {
      std::cout<<"finish all files\n";
      break;
    }

    std::shared_ptr<cvo::RawImage> target_raw(new cvo::RawImage(left));
    std::shared_ptr<cvo::CvoPointCloud> target(new cvo::CvoPointCloud(*target_raw, right, calib, true));
    Eigen::Matrix4f result, init_guess_inv;
    Eigen::Matrix4f identity_init;
   identity_init << 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1; 
    
    init_guess_inv = init_guess.inverse().eval();
    printf("Start align... num_fixed is %d, num_moving is %d\n", source->num_points(), target->num_points());
    cvo_align.function_angle(*source, *target, identity_init);
    std::cout<<"\n\n===========next frame=============\n\n";
   
    source = target;

  }


  return 0;
}
