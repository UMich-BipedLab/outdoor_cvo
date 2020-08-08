#include "utils/CvoPointCloud.hpp"
#include "utils/CvoPixelSelector.hpp"
#include "utils/LidarPointSelector.hpp"
#include "utils/LeGoLoamPointSelection.hpp"
#include "cupointcloud/point_types.h"
#include "cupointcloud/cupointcloud.h"
#include "cukdtree/cukdtree.h"
#include "cvo/gpu_utils.cuh"
#include <chrono>
#include <vector>
#include <memory>
#include <iostream>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace cvo {
  /*   helper functions  */
  static
  __global__
  void init_covariance(perl_registration::cuPointXYZ * points, // mutate
                       int num_points,
                       int * neighbors,
                       int num_neighbors_each_point,
                       // outputs
                       float  * covariance_out,
                       float * eigenvalues_out
                       ) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i > num_points - 1) {
      return;
    }
    if (i == 0)
      printf("i==0 start\n");
    perl_registration::cuPointXYZ & curr_p = points[i];
    if (i == 0)
      printf("i==0: p is %f,%f,%f\n", curr_p.x, curr_p.y, curr_p.z);
    
    Eigen::Vector3f curr_p_vec (curr_p.x, curr_p.y, curr_p.z);
    int * indices = neighbors + i * num_neighbors_each_point;
    
    Eigen::Vector3f mean(0, 0, 0);
    int num_neighbors_in_range = 0;
    for (int j = 0; j < num_neighbors_each_point; j++) {
      auto & neighbor = points[indices[j]];
      Eigen::Vector3f neighbor_vec(neighbor.x, neighbor.y, neighbor.z);
      mean = (mean + neighbor_vec).eval();
      num_neighbors_in_range += 1;
    }
    mean = mean * (1.0f / static_cast<float>(num_neighbors_in_range));
    if (i == 0)
      printf("i==0: mean is %f,%f,%f\n", mean(0),mean(1), mean(2));

    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (int j = 0; j < num_neighbors_each_point; j++) {
      auto & neighbor = points[indices[j]];
      Eigen::Vector3f neighbor_vec(neighbor.x, neighbor.y, neighbor.z);
      Eigen::Vector3f x_minis_mean = neighbor_vec - mean;
      Eigen::Matrix3f temp_cov = x_minis_mean * x_minis_mean.transpose();
      covariance = covariance + temp_cov;
      if (i == 0) {
        //printf("i==0: x_minis_mean is %.3f,%.3f,%.3f\n", x_minis_mean(0),
        //       x_minis_mean(1), x_minis_mean(2));
        printf("i==0: neighbor (%f, %f, %f)\n", neighbor.x, neighbor.y, neighbor.z);
      }
    }
    float * cov_curr = covariance_out + i * 9;
    for (int j = 0; j < 9; j++)
      cov_curr[j] = covariance(j/3, j%3);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(3);
    //es.compute(covariance);
    if (i == 0) {
      printf("covariance is\n %f %f %f\n %f %f %f\n %f %f %f\n",
             covariance(0,0), covariance(0,1), covariance(0,2),
             covariance(1,0), covariance(1,1), covariance(1,2),
             covariance(2,0), covariance(2,1), covariance(2,2));

    }
    /* 
    //auto e_values = es.eigenvalues();
    for (int j = 0; j < 3; j++) {
      eigenvalues_out[j+3*i] = e_values(j);
      
      if (i == 0) {
        printf("i==0: cov_eigenvalue is %.3f,%.3f,%.3f\n", eigenvalues_out[3*i],
               eigenvalues_out[1+3*i], eigenvalues_out[2+3*i]);
      }
      }*/

    /* 
    //PCA in GICP 
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(3);
    es.computeDirect(covariance);
    // Eigen values are sorted
    Eigen::Matrix3f eigen_value_replacement = Eigen::Matrix3f::Zero();
    eigen_value_replacement(0, 0) = 1e-3;
    eigen_value_replacement(1, 1) = 1.0;
    eigen_value_replacement(2, 2) = 1.0;
    covariances.data[pos] = es.eigenvectors() * eigen_value_replacement *
    es.eigenvectors().transpose();
    covariance = covariances.data[pos];
    */
  }

  static
  __global__
  void init_covariance_and_compute_norms(perl_registration::cuPointXYZ * points, // mutate
                       int num_points,
                       int * neighbors,
                       int num_neighbors_each_point,
                       // outputs
                       float * covariance_out,
                       float * eigenvalues_out,
                       float * norms_out,
                       bool * is_cov_degenerate
                       ) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i > num_points - 1) {
      return;
    }
    if (i == 0)
      printf("i==0 start\n");
    perl_registration::cuPointXYZ & curr_p = points[i];
    if (i == 0)
      printf("i==0: p is %f,%f,%f\n", curr_p.x, curr_p.y, curr_p.z);
    Eigen::Vector3f curr_p_vec (curr_p.x, curr_p.y, curr_p.z);

    // find indices for neighbors
    int * indices = neighbors + i * num_neighbors_each_point;
    
    // calculate means from neighbors
    Eigen::Vector3f mean(0, 0, 0);
    int num_neighbors_in_range = 0;
    for (int j = 0; j < num_neighbors_each_point; j++) {
      auto & neighbor = points[indices[j]];
      if (squared_dist(neighbor, curr_p) > 0.55 * 0.55)
        continue;
      Eigen::Vector3f neighbor_vec(neighbor.x, neighbor.y, neighbor.z);
      mean = (mean + neighbor_vec).eval();
      num_neighbors_in_range += 1;
    }
    mean = mean * (1.0f / static_cast<float>(num_neighbors_in_range));
    
    // check if cov is degenerated
    if (num_neighbors_in_range < 10) {
      is_cov_degenerate[i] = true;
      return;
    } else
      is_cov_degenerate[i] = false;

    if (i == 0)
      printf("i==0: mean is %f,%f,%f\n", mean(0),mean(1), mean(2));
    
    // compute covariance
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    // num_neighbors_in_range = 0;
    for (int j = 0; j < num_neighbors_each_point; j++) {
      auto & neighbor = points[indices[j]];
      if (squared_dist(neighbor, curr_p) > 0.55 * 0.55)
        continue;
      Eigen::Vector3f neighbor_vec(neighbor.x, neighbor.y, neighbor.z);
      Eigen::Vector3f x_minis_mean = neighbor_vec - mean;
      Eigen::Matrix3f temp_cov = x_minis_mean * x_minis_mean.transpose();
      covariance = covariance + temp_cov;
      // num_neighbors_in_range += 1;
      if (i == 0) {
        //printf("i==0: x_minis_mean is %.3f,%.3f,%.3f\n", x_minis_mean(0),
        //       x_minis_mean(1), x_minis_mean(2));
        printf("i==0: neighbor (%f, %f, %f)\n", neighbor.x, neighbor.y, neighbor.z);
      }
    }

    covariance = covariance * (1.0f / static_cast<float>(num_neighbors_in_range - 1));

    // assign cov for output
    float * cov_curr = covariance_out + i * 9;
    for (int j = 0; j < 9; j++)
      cov_curr[j] = covariance(j/3, j%3);

    //Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(covariance);
    if (i == 0) {
      printf("covariance is\n %f %f %f\n %f %f %f\n %f %f %f\n",
             covariance(0,0), covariance(0,1), covariance(0,2),
             covariance(1,0), covariance(1,1), covariance(1,2),
             covariance(2,0), covariance(2,1), covariance(2,2));
      //if (es.info() != Eigen::Success) {
      //   printf("covariance eigendecomposition not successful\n");
      //}
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(3);
    es.computeDirect(covariance);

    // get e_values
    // auto e_values = es.eigenvalues();
    for (int j = 0;j<3; j++) eigenvalues_out[j+i*3] = es.eigenvalues()(j);
    
    // get norms
    float * norm_curr = norms_out + i * 3;
    Eigen::Vector3f temp_norm = es.eigenvectors().col(0);
    for (int j=0; j<3; j++) norms_out[j+i*3] = temp_norm(j);
    
    /* 
    //PCA in GICP 
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(3);
    es.computeDirect(covariance);
    // Eigen values are sorted
    Eigen::Matrix3f eigen_value_replacement = Eigen::Matrix3f::Zero();
    eigen_value_replacement(0, 0) = 1e-3;
    eigen_value_replacement(1, 1) = 1.0;
    eigen_value_replacement(2, 2) = 1.0;
    covariances.data[pos] = es.eigenvectors() * eigen_value_replacement *
    es.eigenvectors().transpose();
    covariance = covariances.data[pos];
    */
  }

  __global__ void compute_curvature(perl_registration::cuPointXYZ * points, // mutate
    int num_points,
    int * neighbors,
    int num_neighbors_each_point,
    float * norms_in,
    // outputs
    float * curvatures_out,
    bool * is_cov_degenerate
    ) {

    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i > num_points - 1) {
      return;
    }

    // if cov is degenerated, skip the calculation
    if (is_cov_degenerate[i])
      return;

    // current point
    perl_registration::cuPointXYZ & curr_p = points[i];
    
    // find indices for neighbors
    int * indices = neighbors + i * num_neighbors_each_point;

    // get current norm
    float * norm_curr_ptr = norms_in + i * 3;
    Eigen::Vector3f norm_curr = Eigen::Vector3f::Zero();
    for (int j=0; j<3; j++) norm_curr[j] = norm_curr_ptr[j];

    // get projection
    Eigen::Matrix3f projection = Eigen::Matrix3f::Identity() - 
      norm_curr * norm_curr.transpose();
    Eigen::Vector3f norm_mean = Eigen::Vector3f::Zero();
    Eigen::Matrix3f norm_covariance = Eigen::Matrix3f::Zero();
    
    // calculate projected norm mean
    int num_neighbors_in_range = 0;
    for (int j = 0; j < num_neighbors_each_point; j++) {
      int cur_idx = indices[j];
      auto & neighbor = points[indices[j]]; 
      if (squared_dist(neighbor, curr_p) > 0.55 * 0.55)
        continue;
      Eigen::Vector3f neighbor_norm(norms_in[cur_idx*3], norms_in[cur_idx*3+1], norms_in[cur_idx*3+2]);
      // printf("projection \n %f \n", projection);
      // printf("nieghbor norm \n %f \n", neighbor_norm);
      Eigen::Vector3f norm_temp = Eigen::Vector3f::Zero();
      norm_temp = projection * neighbor_norm;
      norm_mean += norm_temp;
      num_neighbors_in_range+=1;
      // printf("norm mean \n %f \n", norm_mean);
    }
    norm_mean = norm_mean * (1.0f / static_cast<float>(num_neighbors_in_range));
    
    // calculate norm covariance
    for (int j = 0; j < num_neighbors_each_point; j++) {
      int cur_idx = indices[j];
      auto & neighbor = points[indices[j]]; 
      if (squared_dist(neighbor, curr_p) > 0.55 * 0.55)
        continue;
      Eigen::Vector3f neighbor_norm(norms_in[cur_idx*3], norms_in[cur_idx*3+1], norms_in[cur_idx*3+2]);
      Eigen::Vector3f temp = projection * neighbor_norm - norm_mean;
      Eigen::Matrix3f temp_m = temp * temp.transpose();
      norm_covariance = norm_covariance + temp_m;
    }
    norm_covariance = norm_covariance * (1.0f / static_cast<float>(num_neighbors_in_range - 1));

    /* PCA */
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(3);
    es.computeDirect(norm_covariance);

    // get e_values
    // auto e_values = es.eigenvalues();
    for (int j = 0;j<3; j++) curvatures_out[j+i*3] = es.eigenvalues()(j);


    // Eigen::Vector3f norm = norm_curr;
    // Eigen::Vector3f principal_axis = es.eigenvectors().col(2);
    // Eigen::Matrix3f rot;
    // rot.block<1, 3>(0, 0) = norm;
    // rot.block<1, 3>(1, 0) = principal_axis;
    // rot.block<1, 3>(2, 0) = norm.cross(principal_axis);

    // // Covariance like matrix of points if the mean
    // // were 0,0,0
    // //  var(x), cov(x,z), 0
    // //  cov(x,z), var(z), 0
    // //    0,        0,  neighborhood->size()
    // Eigen::Matrix3f Q = Eigen::Matrix3f::Zero();

    // float sum_y = 0;
    // float min_y = -100000;
    // float max_y = 100000;
    // Eigen::Vector3f weighted_sum = Eigen::Vector3f::Zero();
    // for (int i = pos * k; i < (pos + 1) * k; i++) {
    //   // Construct point
    //   Eigen::Vector3f rotated_point = rot * points.data[k_nns.data[i]].toVec();
    //   Eigen::Vector3f temp(rotated_point.x(), rotated_point.z(), 1);
    //   Eigen::Matrix3f temp_m = temp * temp.transpose();
    //   Q = Q + temp_m;

    //   // Construct sum of squares
    //   float sum_squares = rotated_point.x() * rotated_point.x() +
    //   rotated_point.z() * rotated_point.z();

    //   // Accumulate wieghted sum
    //   weighted_sum += sum_squares * temp;

    //   // Accumulate ys
    //   sum_y += rotated_point.y();

    //   // Get min and max y
    //   if (rotated_point.y() < min_y) {
    //   min_y = rotated_point.y();
    //   } else if (rotated_point.y() > max_y) {
    //   max_y = rotated_point.y();
    //   }
    // }

    // // Solve Q * params = weighted_sum
    // //  Since Q will be positive semi definite by
    // //  construction we can use ldlt solver
    // //  which is fast for large and small matrices
    // //  and accurate
    // // TODO(Kevin): Explain what params are and
    // // why theres a -1
    // Q = Q + Eigen::Matrix3f::Identity() * 0.001;
    // Eigen::Matrix3f inv_Q = Inverse(Q);
    // Eigen::Vector3f params = -1 * inv_Q * weighted_sum;
    // //  params = -1 *
    // //  Q.partialPivLu().solve(weighted_sum);

    // // FOR  MATRIX INVERSE ERROR  CHECKING
    // float inv_error = (Q * params + weighted_sum).norm() / weighted_sum.norm();

    // Curvature::Shell shell;
    // float avg_y = sum_y / (float)k;
    // Eigen::Vector3f tfed_centroid;
    // tfed_centroid(0) = -0.5 * params(0);
    // tfed_centroid(1) = avg_y;
    // tfed_centroid(2) = -0.5 * params(1);
    // shell.centroid = rot.transpose() * tfed_centroid;
    // shell.radius =
    // sqrt(0.25f * (params(0) * params(0) + params(1) * params(1)) - params(2));
    // shell.extent = max_y - min_y;
    // shell.param = params;

    // // if(isnan(shell.radius)) {
    // if (pos == 2000) {
    //   printf("Eigen Values %f %f %f\n", es.eigenvalues()(0), es.eigenvalues()(1),
    //   es.eigenvalues()(2));
    //   printf("Radius %f index %d inv error %f \n", shell.radius, pos, inv_error);
    //   printf("Q\n %f %f %f\n %f %f %f\n %f %f %f\n", Q(0, 0), Q(0, 1), Q(0, 2),
    //   Q(1, 0), Q(1, 1), Q(1, 2), Q(2, 0), Q(2, 1), Q(2, 2));
    //   printf("Inv Q\n %f %f %f\n %f %f %f\n %f %f %f\n", inv_Q(0, 0), inv_Q(0, 1),
    //   inv_Q(0, 2), inv_Q(1, 0), inv_Q(1, 1), inv_Q(1, 2), inv_Q(2, 0),
    //   inv_Q(2, 1), inv_Q(2, 2));
    //   printf("params %f %f %f\n", params(0), params(1), params(2));
    //   printf("Weight sum %f %f %f\n", weighted_sum(0), weighted_sum(1),
    //   weighted_sum(2));
    // }

    // out_points.data[pos] = cuPointXYZI(points.data[pos].x, points.data[pos].y,
    //         points.data[pos].z, es.eigenvalues()(2));

    // return;
  }

  static
  void fill_in_pointcloud_covariance(perl_registration::cuPointCloud<perl_registration::cuPointXYZ>::SharedPtr pointcloud_gpu, thrust::device_vector<float> & covariance, thrust::device_vector<float> & eigenvalues ) {
    auto start = std::chrono::system_clock::now();
    perl_registration::cuKdTree<perl_registration::cuPointXYZ> kdtree;
    kdtree.SetInputCloud(pointcloud_gpu);
    const int num_wanted_points = KDTREE_K_SIZE;
    thrust::device_vector<int> indices;

    kdtree.NearestKSearch(pointcloud_gpu, num_wanted_points, indices );
    cudaDeviceSynchronize();

    std::cout<<"Init cov, point size is "<<pointcloud_gpu->size()<<std::endl<<std::flush;

    /*
      thrust::host_vector<int> indices_host = indices;
      for(int i = 0; i < num_wanted_points; i++) {
      std::cout<<indices_host[i+num_wanted_points * (pointcloud_gpu->size()-1)]<<",";
      
      }
      std::cout<<"\n";
    */

    covariance.resize(pointcloud_gpu->size()*9);
    eigenvalues.resize(pointcloud_gpu->size()*3);
    init_covariance<<<(pointcloud_gpu->size() / 512 + 1), 512>>>(
                                                                 thrust::raw_pointer_cast(pointcloud_gpu->points.data()), // mutate
                                                                 pointcloud_gpu->size(),
                                                                 thrust::raw_pointer_cast(indices.data() ),
                                                                 num_wanted_points,
                                                                 thrust::raw_pointer_cast(covariance.data()),
                                                                 thrust::raw_pointer_cast(eigenvalues.data()));
    cudaDeviceSynchronize();
    
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> t_kdtree_search = end-start;
    std::cout<<"kdtree construction and  nn search time is "<<t_kdtree_search.count()<<std::endl;
  }

  static
  void fill_in_pointcloud_curvature(perl_registration::cuPointCloud<perl_registration::cuPointXYZ>::SharedPtr pointcloud_gpu,
    thrust::device_vector<float> & covariance, 
    thrust::device_vector<float> & eigenvalues, 
    thrust::device_vector<float> & norms, 
    thrust::device_vector<float> & curvatures,
    thrust::device_vector<bool> & is_cov_degenerate ) {


    auto start = std::chrono::system_clock::now();
    perl_registration::cuKdTree<perl_registration::cuPointXYZ> kdtree;
    kdtree.SetInputCloud(pointcloud_gpu);
    const int num_wanted_points = KDTREE_K_SIZE;
    thrust::device_vector<int> indices;
    
    kdtree.NearestKSearch(pointcloud_gpu, num_wanted_points, indices );
    cudaDeviceSynchronize();

    std::cout<<"Init cov, point size is "<<pointcloud_gpu->size()<<std::endl<<std::flush;

    /*
      thrust::host_vector<int> indices_host = indices;
      for(int i = 0; i < num_wanted_points; i++) {
      std::cout<<indices_host[i+num_wanted_points * (pointcloud_gpu->size()-1)]<<",";
      
      }
      std::cout<<"\n";
    */

    covariance.resize(pointcloud_gpu->size()*9);
    eigenvalues.resize(pointcloud_gpu->size()*3);
    norms.resize(pointcloud_gpu->size()*3);
    is_cov_degenerate.resize(pointcloud_gpu->size());
    curvatures.resize(pointcloud_gpu->size()*3);

    init_covariance_and_compute_norms<<<(pointcloud_gpu->size() / 512 + 1), 512>>>(
                                                                 thrust::raw_pointer_cast(pointcloud_gpu->points.data()), // mutate
                                                                 pointcloud_gpu->size(),
                                                                 thrust::raw_pointer_cast(indices.data() ),
                                                                 num_wanted_points,
                                                                 thrust::raw_pointer_cast(covariance.data()),
                                                                 thrust::raw_pointer_cast(eigenvalues.data()),
                                                                 thrust::raw_pointer_cast(norms.data()),
                                                                 thrust::raw_pointer_cast(is_cov_degenerate.data()));
    cudaDeviceSynchronize();
    std::cout<<"finished norms"<<std::endl;
    compute_curvature<<<(pointcloud_gpu->size() / 512 + 1), 512>>>(
                thrust::raw_pointer_cast(pointcloud_gpu->points.data()), // mutate
                pointcloud_gpu->size(),
                thrust::raw_pointer_cast(indices.data() ),
                num_wanted_points,
                thrust::raw_pointer_cast(norms.data()),
                // outputs
                thrust::raw_pointer_cast(curvatures.data()),
                thrust::raw_pointer_cast(is_cov_degenerate.data())
                );
    cudaDeviceSynchronize();
    std::cout<<"finished curvatures"<<std::endl;
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> t_kdtree_search = end-start;
    std::cout<<"kdtree construction and  nn search time is "<<t_kdtree_search.count()<<std::endl<<std::flush;
  }

  static
  void compute_covariance(const pcl::PointCloud<pcl::PointXYZI> & pc_raw,
                          thrust::device_vector<float> & covariance_all,
                          thrust::device_vector<float> & eigenvalues_all
                          )   {


    //int num_points = selected_indexes.size();
    int num_points = pc_raw.size();

    // set basic informations for pcl_cloud
    thrust::host_vector<perl_registration::cuPointXYZ> host_cloud;
    //pcl::PointCloud<perl_registration::cuPointXYZ> host_cloud;
    host_cloud.resize(num_points);

    for(int i=0; i<num_points; ++i){
      //auto & curr_p = pc_raw[selected_indexes[i]];
      auto & curr_p = pc_raw[i];
      (host_cloud)[i].x = curr_p.x;
      (host_cloud)[i].y = curr_p.y;
      (host_cloud)[i].z = curr_p.z;
    }
   
    //gpu_cloud->points = host_cloud;
    //auto pc_gpu = std::make_shared<perl_registration::cuPointCloud<perl_registration::cuPointXYZ>>(new perl_registration::cuPointCloud<perl_registration::cuPointXYZ>>);
    perl_registration::cuPointCloud<perl_registration::cuPointXYZ>::SharedPtr pc_gpu (new perl_registration::cuPointCloud<perl_registration::cuPointXYZ>);
    pc_gpu->points = host_cloud;

    fill_in_pointcloud_covariance(pc_gpu, covariance_all, eigenvalues_all);

    return;
  }

  static
  void compute_curvature(const pcl::PointCloud<pcl::PointXYZI> & pc_raw,
                          thrust::device_vector<float> & covariance_all,
                          thrust::device_vector<float> & eigenvalues_all,
                          thrust::device_vector<float> & norms_all,
                          thrust::device_vector<float> & curvatures_all,
                          thrust::device_vector<bool> & is_cov_degenerate
                          )   {


    //int num_points = selected_indexes.size();
    int num_points = pc_raw.size();

    // set basic informations for pcl_cloud
    thrust::host_vector<perl_registration::cuPointXYZ> host_cloud;
    //pcl::PointCloud<perl_registration::cuPointXYZ> host_cloud;
    host_cloud.resize(num_points);

    for(int i=0; i<num_points; ++i){
      //auto & curr_p = pc_raw[selected_indexes[i]];
      auto & curr_p = pc_raw[i];
      (host_cloud)[i].x = curr_p.x;
      (host_cloud)[i].y = curr_p.y;
      (host_cloud)[i].z = curr_p.z;
    }
   
    //gpu_cloud->points = host_cloud;
    //auto pc_gpu = std::make_shared<perl_registration::cuPointCloud<perl_registration::cuPointXYZ>>(new perl_registration::cuPointCloud<perl_registration::cuPointXYZ>>);
    perl_registration::cuPointCloud<perl_registration::cuPointXYZ>::SharedPtr pc_gpu (new perl_registration::cuPointCloud<perl_registration::cuPointXYZ>);
    pc_gpu->points = host_cloud;

    fill_in_pointcloud_curvature(pc_gpu, covariance_all, eigenvalues_all, norms_all, curvatures_all, is_cov_degenerate);
    std::cout<<"finished filling in point cloud curvature"<<std::endl<<std::flush;

    return;
  }
  
  CvoPointCloud::CvoPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr pc,  int beam_num) {
    double intensity_bound = 0.4;
    double depth_bound = 4.0;
    double distance_bound = 75.0;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out (new pcl::PointCloud<pcl::PointXYZI>);
    //std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> pc_out = std::make_unique<pcl::PointCloud<pcl::PointXYZI>>();
    //pcl::PointCloud<pcl::PointXYZI> pc_out;
    std::vector <double> output_depth_grad;
    std::vector <double> output_intenstity_grad;
    std::vector <int> selected_indexes;

    
    int expected_points = 5000;

    /*
    std::vector <float> edge_or_surface;
    LidarPointSelector lps(expected_points, intensity_bound, depth_bound, distance_bound, beam_num);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_edge (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_surface (new pcl::PointCloud<pcl::PointXYZI>);
    lps.edge_detection(pc, pc_out_edge, output_depth_grad, output_intenstity_grad, selected_indexes);    
    lps.legoloam_point_selector(pc, pc_out_surface, edge_or_surface, selected_indexes);    
    *pc_out += *pc_out_edge;
    *pc_out += *pc_out_surface;
    */

    
    random_surface_with_edges(pc, expected_points, intensity_bound, depth_bound, distance_bound, beam_num,
                              output_depth_grad, output_intenstity_grad, selected_indexes);
    std::cout<<"compute covariance\n";
    thrust::device_vector<float> cov_all, eig_all, norm_all, curv_all;
    thrust::device_vector<bool> is_cov_degenerate_gpu;
    compute_curvature(*pc, cov_all, eig_all, norm_all, curv_all, is_cov_degenerate_gpu);
    std::cout<<"finished computing curvature"<<std::endl;
    // compute_covariance(*pc, cov_all, eig_all);
    std::unique_ptr<thrust::host_vector<float>> cov(new thrust::host_vector<float>(cov_all));
    std::unique_ptr<thrust::host_vector<float>> eig(new thrust::host_vector<float>(eig_all));
    std::unique_ptr<thrust::host_vector<float>> norm(new thrust::host_vector<float>(norm_all));
    std::unique_ptr<thrust::host_vector<float>> curv(new thrust::host_vector<float>(curv_all));
    std::unique_ptr<thrust::host_vector<bool>> is_cov_degenerate_host(new thrust::host_vector<bool>(is_cov_degenerate_gpu));
    
    // fill in class members
    // num_points_ = selected_indexes.size();
    num_points_ = 0;
    for (int j = 0; j < selected_indexes.size(); j++){
      if((*is_cov_degenerate_host)[selected_indexes[j]] == false)
        num_points_+=1;
    }
    num_classes_ = 0;
    
    // features_ = Eigen::MatrixXf::Zero(num_points_, 1);
    feature_dimensions_ = 1;
    features_.resize(num_points_, feature_dimensions_);
    normals_.resize(num_points_,3);
    curvatures_.resize(num_points_,3);
    covariance_.resize(num_points_ * 9);
    eigenvalues_.resize(num_points_*3);
    //eigenvalues_.resize(num_points_ * 3);
    //types_.resize(num_points_, 2);


    std::ofstream e_value_max("e_value_max.txt");
    std::ofstream e_value_min("e_value_min.txt");
    
    int actual_i = 0;
    // assert(num_points_ == selected_indexes.size());
    for (int i = 0; i < selected_indexes.size() ; i++) {
      if ((*is_cov_degenerate_host)[selected_indexes[i]]) continue;
      int id_pc_in = selected_indexes[i];
      Vec3f xyz;
      //xyz << pc_out->points[i].x, pc_out->points[i].y, pc_out->points[i].z;
      xyz << pc->points[id_pc_in].x, pc->points[id_pc_in].y, pc->points[id_pc_in].z;
      positions_.push_back(xyz);
      features_(actual_i, 0) = pc->points[id_pc_in].intensity;
      
      memcpy(covariance_.data() + actual_i * 9, cov->data() + id_pc_in * 9, sizeof(float)*9);
      memcpy(eigenvalues_.data() + actual_i * 3, eig->data() + id_pc_in * 3, sizeof(float)*3);
      memcpy(curvatures_.data() + actual_i * 3, curv->data() + id_pc_in * 3, sizeof(float)*3);

      // Eigen::Matrix3f cov_curr = Eigen::Map<Eigen::Matrix3f>(&covariance_.data()[i*9]);
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov_curr);
      // Eigen::Vector3f e_values = es.eigenvalues();
      // for (int j = 0; j < 3; j++) {
      //   eigenvalues_[j+i*3] = e_values(j);
      //   curvatures_(i,j) = 
      // }
      // std::cout<<curvatures_(actual_i,0)<<", "<<curvatures_(actual_i,1)<<","<<curvatures_(actual_i,2)<<std::endl;  
      // std::cout<<*(curv->data() + id_pc_in * 3)<<", "<<*(curv->data() + id_pc_in * 3+1)<<","<<*(curv->data() + id_pc_in * 3+2)<<std::endl;
      // std::cout<<"--------"<<std::endl;

      /*
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(3);
      es.computeDirect(cov_curr);
      Eigen::Matrix3f eigen_value_replacement = Eigen::Matrix3f::Zero();
      eigen_value_replacement(0, 0) = 1e-3;
      eigen_value_replacement(1, 1) = 1.0;
      eigen_value_replacement(2, 2) = 1.0;
      cov_curr = es.eigenvectors() * eigen_value_replacement *
        es.eigenvectors().transpose();
      //covariance = covariances.data[pos];
      eigenvalues_[i*3]=1e-3;
      eigenvalues_[i*3+1]=1.0;
      eigenvalues_[i*3+2]=1.0;
      */

      // e_value_max << e_values(2) << std::endl;
      // e_value_min << e_values(0) << std::endl;
      
      //memcpy(eigenvalues_.data() + i * 3, eig->data() + id_pc_in * 3, sizeof(float)*3);

      actual_i++;
    }

    e_value_min.close();
    e_value_max.close();
    
    std::cout<<"Construct Cvo PointCloud, num of points is "<<num_points_<<" from "<<pc->size()<<" input points "<<std::endl;    
    //write_to_intensity_pcd("kitti_lidar.pcd");

  }

  

}
