#ifndef KDSEARCH_H
#define KDSEARCH_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>

#include "trackingbox.h"

class kdsearch
{
public:
  kdsearch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, 
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model,
	   Eigen::Affine3f trans_matrix);
  ~kdsearch();
  Eigen::Affine3f trans;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud_kd;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model_kd;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud_kd_trans;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model_kd_trans;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_hand;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr hand_cloud_rgb;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model_rgb;
  
  void search();
private:
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  const int K = 100;
  const float eps = 0.02;
  
  std::vector<int> index_object;
};

#endif