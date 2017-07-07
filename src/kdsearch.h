#ifndef KDSEARCH_H
#define KDSEARCH_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>

#include "trackingbox.h"

class kdsearch
{
public:
  kdsearch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, 
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model);
  ~kdsearch();
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud_kd;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model_kd;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_hand;
  
  void search();
private:
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  const int K = 1;
  
  std::vector<int> index_object;
};

#endif