#include "kdsearch.h"

kdsearch::kdsearch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, 
		   pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model)
{
  hand_cloud_kd.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  transformed_model_kd.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::cout << "Reset cloud !" << std::endl;
   std::cout  <<transformed_model->points.size()<<std::endl;
  std::cout  <<hand_cloud->points.size()<<std::endl;
  /*
  for (int i = 0; i < hand_cloud->points.size(); i++)
     {hand_cloud_kd -> points.push_back(hand_cloud -> points[i]);}
     
  for (int j = 0; j < transformed_model -> size(); j++)
     {transformed_model_kd -> points.push_back(transformed_model -> points[j]);}
  */
  hand_cloud_kd = hand_cloud;
  transformed_model_kd = transformed_model;
  std::cout << "The size of search area is: " << hand_cloud_kd -> size() << std::endl;
  std::cout << "The number of points to search is: " << transformed_model_kd -> size() << std::endl;
  
  kdtree.setInputCloud(hand_cloud_kd);
  
}


kdsearch::~kdsearch()
{}


void kdsearch::search()
{
  int cloud_size = hand_cloud_kd -> size();
  raw_hand.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < cloud_size; i++)
  {
//     pointKNNSquareDistance.clear();
//     pointIdxKNNSearch.clear();
    pcl::PointXYZRGB searchPoint = hand_cloud_kd -> points[i];
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquareDistance(K);
    int num_found = kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquareDistance);
    if (num_found)
    {
       index_object.push_back(1);
       raw_hand -> push_back(hand_cloud_kd -> points[i]);
    }
    else
    {
       index_object.push_back(0);
    }
  }
}
