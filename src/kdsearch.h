#ifndef KDSEARCH_H
#define KDSEARCH_H

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "trackingbox.h"
#include "boundbox.h"

// std::vector<cv::Point2i> TwoDim_coord;
extern std::vector<cv::Point2i> after_KDsearch;
extern std::vector<cv::Point2i> RawHand_coord;

class kdsearch
{
public:
  kdsearch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, 
           pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model,
	   Eigen::Affine3f trans_matrix);
  ~kdsearch();
  void search();
  void largest_cluster_filter(); 
  
  Eigen::Affine3f trans;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud_kd;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model_kd;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud_kd_trans;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model_kd_trans;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_hand;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_hand_filtered;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr hand_cloud_rgb;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_model_rgb;
  
  std::vector<cv::Point2i> temp1;
   
private:
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
 
  const int K = 1;
  const float eps = 5e-5;      // 0.02 for radius search 1.6e-4, 1.2e-4 for original camera distance, 1e-4
  const float cluster_tolerance = 0.01;         // 0.01, 0.004 for original camera distance
  
  std::vector<int> index_object;
};

#endif