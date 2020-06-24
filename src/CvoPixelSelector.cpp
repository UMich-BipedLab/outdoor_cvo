/**
 * This file is part of DSO.
 * 
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSO. If not, see <http://www.gnu.org/licenses/>.
 */



#include <iostream>
#include <cstring>
#include <cmath>

#include <vector>
#include <string>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/principal_curvatures.h>
#include "utils/CvoPixelSelector.hpp"


namespace cvo
{

  const float setting_gradDownweightPerLevel  = 0.75;
  const bool setting_selectDirectionDistribution = false;
  const float setting_minGradHistCut  = 0.5;
  const float setting_minGradHistAdd = 7;

  PixelSelector::PixelSelector(int w_, int h_) :
    w(w_), h(h_)
  {
    randomPattern.resize(w*h);
    std::srand(3141592);	// want to be deterministic.
    for(int i=0;i<w*h;i++) randomPattern[i] = rand() & 0xFF;

    currentPotential=3;


    gradHist .resize( 100*(1+w/32)*(1+h/32));
    ths.resize( (w/32)*(h/32)+100 );
    thsSmoothed.resize( (w/32)*(h/32)+100 );
  
    allowFastCornerSelector=false;
    gradHistFrame=0;
  }

  PixelSelector::~PixelSelector() {
  }

  static int computeHistQuantil(int* hist, float below) {
    int th = hist[0]*below+0.5f;
    for(int i=0;i<90;i++)
    {
      th -= hist[i+1];
      if(th<0) return i;
    }
    return 90;
  }


  void PixelSelector::makeHists(const RawImage & raw_image)
  {	
    gradHistFrame = &raw_image;
    const float * mapmax0 = raw_image.gradient_square().data();

    int w = raw_image.color().cols;
    int h = raw_image.color().rows;

    int w32 = w/32;
    int h32 = h/32;
    thsStep = w32;

    for(int y=0;y<h32;y++)
      for(int x=0;x<w32;x++)
      {	
        auto map0 = mapmax0+32*x+32*y*w;
        auto hist0 = gradHist.data();// + 50*(x+y*w32);
        memset(hist0,0,sizeof(int)*50);
			
        for(int j=0;j<32;j++)
          for(int i=0;i<32;i++) {
            int it = i+32*x;
            int jt = j+32*y;

            if(it>w-2 || jt>h-2 || it<1 || jt<1) continue;
            int g = sqrtf(map0[i+j*w]);

            if(g>48) g=48;
            hist0[g+1]++;
            hist0[0]++;
          }
			
        ths[x+y*w32] = computeHistQuantil(hist0,setting_minGradHistCut) + setting_minGradHistAdd;
			
      }

    for(int y=0;y<h32;y++)
      for(int x=0;x<w32;x++)
      {
        float sum=0,num=0;
        if(x>0)
        {
          if(y>0) 	{num++; 	sum+=ths[x-1+(y-1)*w32];}
          if(y<h32-1) {num++; 	sum+=ths[x-1+(y+1)*w32];}
          num++; sum+=ths[x-1+(y)*w32];
        }

        if(x<w32-1)
        {
          if(y>0) 	{num++; 	sum+=ths[x+1+(y-1)*w32];}
          if(y<h32-1) {num++; 	sum+=ths[x+1+(y+1)*w32];}
          num++; sum+=ths[x+1+(y)*w32];
        }

        if(y>0) 	{num++; 	sum+=ths[x+(y-1)*w32];}
        if(y<h32-1) {num++; 	sum+=ths[x+(y+1)*w32];}
        num++; sum+=ths[x+y*w32];

        thsSmoothed[x+y*w32] = (sum/num) * (sum/num);

      }




  }
  int PixelSelector::makeHeatMaps(const RawImage & raw_image,
                                  float density, 
                                  // output
                                  float * map_out,
                                  std::vector<Vec2i, Eigen::aligned_allocator<Vec2i>> & output_uv,
                                  // default inputs
                                  int customized_potential,
                                  int recursionsLeft, bool plot, float thFactor)
  {
    float numHave=0;
    float numWant=density;
    float quotia;
    if (customized_potential!=-1) currentPotential = customized_potential;
    int idealPotential = currentPotential;

    //	if(setting_pixelSelectionUseFast>0 && allowFastCornerS)
    //	{
    //		memset(map_out, 0, sizeof(float)*wG[0]*hG[0]);
    //		std::vector<cv::KeyPoint> pts;
    //		cv::Mat img8u(hG[0],wG[0],CV_8U);
    //		for(int i=0;i<wG[0]*hG[0];i++)
    //		{
    //			float v = ptr_fr->dI[i][0]*0.8;
    //			img8u.at<uchar>(i) = (!std::isfinite(v) || v>255) ? 255 : v;
    //		}
    //		cv::FAST(img8u, pts, setting_pixelSelectionUseFast, true);
    //		for(unsigned int i=0;i<pts.size();i++)
    //		{
    //			int x = pts[i].pt.x+0.5;
    //			int y = pts[i].pt.y+0.5;
    //			map_out[x+y*wG[0]]=1;
    //			numHave++;
    //		}
    //
    //		printf("FAST selection: got %f / %f!\n", numHave, numWant);
    //		quotia = numWant / numHave;
    //	}
    //	else
    {
      // the number of selected pixels behaves approximately as
      // K / (pot+1)^2, where K is a scene-dependent constant.
      // we will allow sub-selecting pixels by up to a quotia of 0.25, otherwise we will re-select.

      if(&raw_image != gradHistFrame)
        makeHists(raw_image);

      // select!
      std::cout<<"makeHeatMaps: currentPotential is "<<currentPotential<<", thFactor is "<<thFactor<<std::endl;
      Eigen::Vector3i n = this->select(raw_image,  currentPotential, thFactor, map_out, output_uv);

      // sub-select!
      numHave = static_cast<float>(n[0]+n[1]+n[2]);
      // std::cout<<"numHave n is "<<n.transpose()<<std::endl;
      quotia = numWant / numHave;


      // by default we want to over-sample by 40% just to be sure.
      float K = numHave * (currentPotential+1) * (currentPotential+1);
      idealPotential = sqrtf(K/numWant)-1;	// round down.
      if(idealPotential<1) idealPotential=1;
      // printf("PixelSelector: recursionsLeft %d, quotia %f, numHave %d / numWant %d, idealPotential is %f\n",recursionsLeft,  quotia, numHave, numWant, idealPotential);
      if( recursionsLeft>0 && quotia > 1.25 && currentPotential>1)
      {
        //re-sample to get more points!
        // potential needs to be smaller
        if(idealPotential>=currentPotential)
          idealPotential = currentPotential-1;

        //		printf("PixelSelector: have %.2f%%, need %.2f%%. RESAMPLE with pot %d -> %d.\n",
        //				100*numHave/(float)(wG[0]*hG[0]),
        //				100*numWant/(float)(wG[0]*hG[0]),
        //				currentPotential,
        //				idealPotential);
        currentPotential = idealPotential;
        return makeHeatMaps(raw_image,density, map_out, output_uv, recursionsLeft-1, plot,thFactor);
      }
      else if(recursionsLeft>0 && quotia < 0.25)
      {
        // re-sample to get less points!

        if(idealPotential<=currentPotential)
          idealPotential = currentPotential+1;

        //		printf("PixelSelector: have %.2f%%, need %.2f%%. RESAMPLE with pot %d -> %d.\n",
        //				100*numHave/(float)(wG[0]*hG[0]),
        //				100*numWant/(float)(wG[0]*hG[0]),
        //				currentPotential,
        //				idealPotential);
        currentPotential = idealPotential;
        return makeHeatMaps(raw_image, density,  map_out, output_uv, recursionsLeft-1, plot,thFactor);

      }
    }

    int numHaveSub = numHave;
    if(quotia < 0.95)
    {
      int wh=raw_image.color().total();
      int rn=0;
      unsigned char charTH = 255*quotia;
      for(int i=0;i<wh;i++)
      {
        if(map_out[i] != 0)
        {
          if(randomPattern[rn] > charTH )
          {
            map_out[i]=0;
            numHaveSub--;
          }
          rn++;
        }
      }
    }

    currentPotential = idealPotential;

    return numHaveSub;
  }



  Eigen::Vector3i PixelSelector::select(const RawImage & raw_image, 
                                        int pot, float thFactor,
                                        // outputs
                                        float * map_out,
                                        std::vector<Vec2i, Eigen::aligned_allocator<Vec2i>> & output_uv 
                                        )
  {


    auto map0 = raw_image.intensity().data(); //ptr_fr->dI;
    auto grad0 = raw_image.gradient();
    auto mapmax0 = raw_image.gradient_square().data();
    //float * mapmax0 = ptr_fr->abs_squared_grad[0];
    //float * mapmax1 = ptr_fr->abs_squared_grad[1];
    //float * mapmax2 = ptr_fr->abs_squared_grad[2];

    int w = raw_image.color().cols;
    int w1 = w/2;
    int w2 = w/4;
    int h = raw_image.color().rows;
    // memset(map_out, 0, h * w * sizeof(float));
    // output_uv.clear();
      
    const Vec2f directions[16] = {
                                  Vec2f(0,    1.0000),
                                  Vec2f(0.3827,    0.9239),
                                  Vec2f(0.1951,    0.9808),
                                  Vec2f(0.9239,    0.3827),
                                  Vec2f(0.7071,    0.7071),
                                  Vec2f(0.3827,   -0.9239),
                                  Vec2f(0.8315,    0.5556),
                                  Vec2f(0.8315,   -0.5556),
                                  Vec2f(0.5556,   -0.8315),
                                  Vec2f(0.9808,    0.1951),
                                  Vec2f(0.9239,   -0.3827),
                                  Vec2f(0.7071,   -0.7071),
                                  Vec2f(0.5556,    0.8315),
                                  Vec2f(0.9808,   -0.1951),
                                  Vec2f(1.0000,    0.0000),
                                  Vec2f(0.1951,   -0.9808)};

    memset(map_out,0,w*h*sizeof(PixelSelectorStatus));

    float dw1 = setting_gradDownweightPerLevel;
    float dw2 = dw1*dw1;


    int n3=0, n2=0, n4=0;
    for(int y4=0;y4<h;y4+=(4*pot))
      for(int x4=0;x4<w;x4+=(4*pot))
      {
        int my3 = std::min((4*pot), h-y4);
        int mx3 = std::min((4*pot), w-x4);
        int bestIdx4=-1; float bestVal4=0;
        Vec2f dir4 = directions[randomPattern[n2] & 0xF];
        for(int y3=0;y3<my3;y3+=(2*pot))
          for(int x3=0;x3<mx3;x3+=(2*pot))
          {
            int x34 = x3+x4;
            int y34 = y3+y4;
            int my2 = std::min((2*pot), h-y34);
            int mx2 = std::min((2*pot), w-x34);
            int bestIdx3=-1; float bestVal3=0;
            Vec2f dir3 = directions[randomPattern[n2] & 0xF];
            for(int y2=0;y2<my2;y2+=pot)
              for(int x2=0;x2<mx2;x2+=pot)
              {
                int x234 = x2+x34;
                int y234 = y2+y34;
                int my1 = std::min(pot, h-y234);
                int mx1 = std::min(pot, w-x234);
                int bestIdx2=-1; float bestVal2=0;
                Vec2f dir2 = directions[randomPattern[n2] & 0xF];
                for(int y1=0;y1<my1;y1+=1)
                  for(int x1=0;x1<mx1;x1+=1)
                  {
                    assert(x1+x234 < w);
                    assert(y1+y234 < h);
                    int idx = x1+x234 + w*(y1+y234);
                    int xf = x1+x234;
                    int yf = y1+y234;

                    if(xf<4 || xf>=w-5 || yf<4 || yf>h-4) continue;


                    float pixelTH0 = thsSmoothed[(xf>>5) + (yf>>5) * thsStep];
                    float pixelTH1 = pixelTH0*dw1;
                    float pixelTH2 = pixelTH1*dw2;


                    float ag0 = mapmax0[idx];
                    if(ag0 > pixelTH0*thFactor)
                    {
                      //Vec2f ag0d = map0[idx].tail<2> ();
                      Vec2f ag0d = grad0[idx];
                      float dirNorm = fabsf((float)(ag0d.dot(dir2)));
                      if(!setting_selectDirectionDistribution) dirNorm = ag0;

                      if(dirNorm > bestVal2)
                      { bestVal2 = dirNorm; bestIdx2 = idx; bestIdx3 = -2; bestIdx4 = -2;}
                    }
                    if(bestIdx3==-2) continue;

                    // float ag1 = mapmax1[(int)(xf*0.5f+0.25f) + (int)(yf*0.5f+0.25f)*w1];
                    // if(ag1 > pixelTH1*thFactor)
                    // {
                    // 	Vec2f ag0d = map0[idx].tail<2>();
                    // 	float dirNorm = fabsf((float)(ag0d.dot(dir3)));
                    // 	if(!setting_selectDirectionDistribution) dirNorm = ag1;

                    // 	if(dirNorm > bestVal3)
                    // 	{ bestVal3 = dirNorm; bestIdx3 = idx; bestIdx4 = -2;}
                    // }
                    // if(bestIdx4==-2) continue;

                    // float ag2 = mapmax2[(int)(xf*0.25f+0.125) + (int)(yf*0.25f+0.125)*w2];
                    // if(ag2 > pixelTH2*thFactor)
                    // {
                    // 	Vec2f ag0d = map0[idx].tail<2>();
                    // 	float dirNorm = fabsf((float)(ag0d.dot(dir4)));
                    // 	if(!setting_selectDirectionDistribution) dirNorm = ag2;

                    // 	if(dirNorm > bestVal4)
                    // 	{ bestVal4 = dirNorm; bestIdx4 = idx; }
                    // }
                  }

                if(bestIdx2>0)
                {
                  map_out[bestIdx2] = 1;
                  Vec2i uv;
                  uv << bestIdx2 % w , bestIdx2 / w;
                  output_uv.push_back(uv);
                  bestVal3 = 1e10;
                  n2++;
                }
              }

            // if(bestIdx3>0)
            // {
            // 	map_out[bestIdx3] = 2;
            // 	bestVal4 = 1e10;
            // 	n3++;
            // }
          }

        // if(bestIdx4>0)
        // {
        // 	map_out[bestIdx4] = 4;
        // 	n4++;
        // }
      }

    // printf("[select] n2 is %d\n", n2);
    return Eigen::Vector3i(n2,n3,n4);
  }



  void select_pixels(const RawImage & raw_image,
                     int num_want,
                     // output
                     std::vector<Vec2i, Eigen::aligned_allocator<Vec2i>> & output_uv ) {
    PixelSelector selector(raw_image.color().cols, raw_image.color().rows);
    std::vector<float> heat_map(raw_image.color().total(), 0);
    selector.makeHeatMaps(raw_image,static_cast<float> (num_want), heat_map.data(), output_uv, 5, 0);
    
    bool debug_plot = true;
    if (debug_plot) {
      std::cout<<"Number of selected points is "<<output_uv.size()<<"\n";
      cv::Mat heatmap(raw_image.color().rows, raw_image.color().cols, CV_32FC1, heat_map.data());
      int w = heatmap.cols;
      int h = heatmap.rows;
      for (int i = 0; i < output_uv.size(); i++) {

        cv::circle(heatmap, cv::Point( output_uv[i](0), output_uv[i](1) ), 1, cv::Scalar(255, 0 ,0), 1);

        
      }
      //cv::imshow("heat map", heatmap);
      //cv::waitKey(200);
      cv::imwrite("heatmap.png", heatmap);
      
    }
  }

  void edge_detection(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
                     int num_want,
                     double intensity_bound, 
                     double depth_bound,
                     double distance_bound,
                     // output
                     pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out,
                     std::vector <double> & output_depth_grad,
                     std::vector <double> & output_intenstity_grad,
                     pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curv_out) {

    int beam_num = 64;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, *pc_in, indices);
    int num_points = pc_in->points.size();
    int previous_quadrant = get_quadrant(pc_in->points[0]);
    int ring_num = 0;

    // #ifdef IS_USING_NORMALS
      // calculate surface normals
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> ne;
      ne.setInputCloud(pc_in);
      // Create an empty kdtree representation, and pass it to the normal estimation object.
      // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
      ne.setSearchMethod(tree);
      // Use all neighbors in a sphere of radius 50cm
      ne.setRadiusSearch(1);
      // ne.setKSearch(30);
      // ne.setInputCloud(pc_in);
      ne.compute(*normals);

      // Setup the principal curvatures computation
      pcl::PrincipalCurvaturesEstimation<pcl::PointXYZI, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;

      // Provide the original point cloud (without normals)
      principalCurvaturesEstimation.setInputCloud(pc_in);

      // Provide the point cloud with normals
      principalCurvaturesEstimation.setInputNormals(normals);

      // Use the same KdTree from the normal estimation
      principalCurvaturesEstimation.setSearchMethod (tree);
      principalCurvaturesEstimation.setRadiusSearch(1.0);

      // Actually compute the principal curvatures
      pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr principalCurvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
      principalCurvaturesEstimation.compute (*principalCurvatures);

      std::cout << "output points.size (): " << principalCurvatures->points.size () << std::endl;

      // Display and retrieve the shape context descriptor vector for the 0th point.
      std::cout<<"descriptor: "<<std::endl;
      pcl::PrincipalCurvatures descriptor = principalCurvatures->points[0];
      std::cout << descriptor << std::endl;

      /*
        ----------visualize normals----------
      */
      // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      // viewer.addPointCloudNormals<pcl::PointXYZI,pcl::Normal>(pc_in, normals,10,0.1, "normals1");
      // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals1");
      // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "normals1");
      // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> rgb2 (pc_in, 0, 255, 0); //This will display the point cloud in green (R,G,B)
      // viewer.addPointCloud<pcl::PointXYZI> (pc_in, rgb2, "cloud_RGB2");
      // // viewer.addPointCloud<pcl::PointXYZI>(pc_in, "original_cloud");
      // while (!viewer.wasStopped ())
      // {
      //   viewer.spinOnce ();
      // }
    // #endif

    for(int i = 1; i<num_points; i++) {      
      int quadrant = get_quadrant(pc_in->points[i]);
      if(quadrant == 1 && previous_quadrant == 4 && ring_num < beam_num-1){
        ring_num += 1;
        continue;
      }

      // select points
      const auto& point_l = pc_in->points[i-1];
      const auto& point = pc_in->points[i];
      const auto& point_r = pc_in->points[i+1];
      
      double depth_grad = std::max((point_l.getVector3fMap()-point.getVector3fMap()).norm(),
                      (point.getVector3fMap()-point_r.getVector3fMap()).norm());
      
      double intenstity_grad = std::max(
                              std::abs( point_l.intensity - point.intensity ),
                              std::abs( point.intensity - point_r.intensity ));

      if( (intenstity_grad > intensity_bound || depth_grad > depth_bound) 
           && (point.intensity > 0.0 && !isnan(normals->points[i].normal_x)) 
           && ((point.x!=0.0) && (point.y!=0.0) && (point.z!=0.0)) //){
           && (sqrt(point.x*point.x+point.y*point.y+point.z*point.z) < distance_bound)
           && !isnan(principalCurvatures->points[i].pc1)){
          // std::cout << "points: " << point.x << ", " << point.y << ", " << point.z << ", " << point.intensity << std::endl;
          pc_out->push_back(pc_in->points[i]);
          output_depth_grad.push_back(depth_grad);
          output_intenstity_grad.push_back(intenstity_grad);
          curv_out->push_back(principalCurvatures->points[i]);
      }

      previous_quadrant = quadrant;      
    }

    // visualize
    // pcl::visualization::PCLVisualizer input_viewer ("Input Point Cloud Viewer");
    // input_viewer.addPointCloud<pcl::PointXYZI> (pc_in, "frame0");
    // while (!input_viewer.wasStopped ())
    // {
    //     input_viewer.spinOnce ();
    // }
    // pcl::visualization::PCLVisualizer output_viewer ("Output Point Cloud Viewer");
    // output_viewer.addPointCloud<pcl::PointXYZI> (pc_out, "frame0");
    // while (!output_viewer.wasStopped ())
    // {
    //     output_viewer.spinOnce ();
    // }
    
    // pcl::io::savePCDFile("input.pcd", *pc_in);
    // pcl::io::savePCDFile("output.pcd", *pc_out);
  }

  void edge_detection(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
                     const std::vector<int> & semantic_in,
                     int num_want,
                     double intensity_bound, 
                     double depth_bound,
                     double distance_bound,
                     // output
                     pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out,
                     std::vector <double> & output_depth_grad,
                     std::vector <double> & output_intenstity_grad,
                     std::vector<int> & semantic_out) {

    int beam_num = 64;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*pc_in, *pc_in, indices);
    int num_points = pc_in->points.size();
    int previous_quadrant = get_quadrant(pc_in->points[0]);
    int ring_num = 0;

    for(int i = 1; i<num_points; i++) {   
      if(semantic_in[i]==-1){
        // exclude unlabeled points
        continue;
      }   
      int quadrant = get_quadrant(pc_in->points[i]);
      if(quadrant == 1 && previous_quadrant == 4 && ring_num < beam_num-1){
        ring_num += 1;
        continue;
      }

      // select points
      const auto& point_l = pc_in->points[i-1];
      const auto& point = pc_in->points[i];
      const auto& point_r = pc_in->points[i+1];
      
      double depth_grad = std::max((point_l.getVector3fMap()-point.getVector3fMap()).norm(),
                      (point.getVector3fMap()-point_r.getVector3fMap()).norm());
      
      double intenstity_grad = std::max(
                              std::abs( point_l.intensity - point.intensity ),
                              std::abs( point.intensity - point_r.intensity ));

      if( (intenstity_grad > intensity_bound || depth_grad > depth_bound) 
           && (point.intensity > 0.0) 
           && ((point.x!=0.0) && (point.y!=0.0) && (point.z!=0.0)) //){
           && (sqrt(point.x*point.x+point.y*point.y+point.z*point.z) < distance_bound)){
          pc_out->push_back(pc_in->points[i]);
          output_depth_grad.push_back(depth_grad);
          output_intenstity_grad.push_back(intenstity_grad);
          semantic_out.push_back(semantic_in[i]);
      }

      previous_quadrant = quadrant;      
    }
  }

  void laserCloudHandler(pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
                        int num_want,
                        double intensity_bound, 
                        double depth_bound,
                        // output
                        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out,
                        std::vector <double> & output_depth_grad,
                        std::vector <double> & output_intenstity_grad)
{
  int N_SCANS = 64;
  std::vector<int> scanStartInd(N_SCANS, 0);
  std::vector<int> scanEndInd(N_SCANS, 0);
  
  // double timeScanCur = laserCloudMsg->header.stamp.toSec();
  // pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  // pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pc_in, *pc_in, indices);
  int cloudSize = pc_in->points.size();
  float startOri = -atan2(pc_in->points[0].y, pc_in->points[0].x);
  float endOri = -atan2(pc_in->points[cloudSize - 1].y,
                        pc_in->points[cloudSize - 1].x) + 2 * M_PI;




}

  int get_quadrant(pcl::PointXYZI point){
    int res = 0;
    float x = point.x;
    float y = point.y;
    if(x > 0 && y >= 0){res = 1;}
    else if(x <= 0 && y > 0){res = 2;}
    else if(x < 0 && y <= 0){res = 3;}
    else if(x >= 0 && y < 0){res = 4;}
    return res;
    }
}