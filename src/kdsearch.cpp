#include "kdsearch.h"
std::vector<cv::Point2i> RawHand_coord;
std::vector<cv::Point2i> after_KDsearch;

kdsearch::kdsearch(pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud, 
		   pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model,
		   Eigen::Affine3f trans_matrix)
{
  hand_cloud_kd.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  transformed_model_kd.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  hand_cloud_kd_trans.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  transformed_model_kd_trans.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  
  temp1.clear();
  trans = trans_matrix;
  
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
//   std::cout  << "The transformed matrix is: " << trans.data() << std::endl;
  pcl::transformPointCloud(*hand_cloud, *hand_cloud_kd_trans, trans_matrix.inverse());
  pcl::transformPointCloud(*transformed_model, *transformed_model_kd_trans, trans_matrix.inverse());
  
  std::cout << "The size of search area is: " << hand_cloud_kd_trans -> size() << std::endl;
  std::cout << "The number of points to search is: " << transformed_model_kd_trans -> size() << std::endl;
  
  kdtree.setInputCloud(transformed_model_kd_trans);
//   kdtree.setEpsilon(eps);
  std::cout << "Kd tree already set !" << std::endl;
}


kdsearch::~kdsearch()
{}


void kdsearch::search()
{
//   std::cout << "search epsilon precision is: " << kdtree.getEpsilon() << std::endl ;
  temp1.clear();
  after_KDsearch.clear(); 
  int cloud_size = hand_cloud_kd_trans -> size();
  raw_hand.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < cloud_size; i++)
  {
    pcl::PointXYZRGB searchPoint = hand_cloud_kd_trans -> points[i];
    std::vector<int> pointIdxKNNSearch(K);
    std::vector<float> pointKNNSquareDistance(K);
    int num_found = kdtree.nearestKSearch(searchPoint, K, pointIdxKNNSearch, pointKNNSquareDistance);
    
//     std::vector<int> pointIdxRadiusSearch;
//     std::vector<float> pointRadiusSquaredDistance;
//     int num_found = kdtree.radiusSearch (searchPoint, eps , pointIdxRadiusSearch, pointRadiusSquaredDistance);
//     std::cout << "K distance is: " << pointKNNSquareDistance[0] << std::endl;
    if (pointKNNSquareDistance[0] <= eps)
    {
        index_object.push_back(1);
//        std::cout << "current searched point is " << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << std::endl;
       /*
       for (size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
             std::cout << "    "  <<   transformed_model_kd->points[  pointIdxKNNSearch[i] ].x 
                       << " " << transformed_model_kd->points[  pointIdxKNNSearch[i] ].y 
                       << " " << transformed_model_kd->points[  pointIdxKNNSearch[i] ].z 
                       << " (squared distance: " << pointKNNSquareDistance[i] << ")" << std::endl;
       */
//        std::cout << "The largest error is: " << *std::max_element( pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end()) << std::endl;
//        std::cout << "The smallest error is " << *std::min_element( pointRadiusSquaredDistance.begin(), pointRadiusSquaredDistance.end()) << std::endl;
       // change the distance threshold
    }
    else
    {
       raw_hand -> points.push_back(hand_cloud_kd -> points[i]);
       index_object.push_back(0);
       temp1.push_back(TwoDim_coord[i]);
       after_KDsearch.push_back(TwoDim_coord[i]);
    }
  }
  std::cout << "There are " << raw_hand -> points.size() << " points in raw hand."<< std::endl;
}


void kdsearch::largest_cluster_filter()
{
  RawHand_coord.clear();
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree -> setInputCloud(raw_hand);
  
  std::vector<pcl::PointIndices>  cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(20);
  ec.setMaxClusterSize(70000);   // 30000 for original camera set
  ec.setSearchMethod(tree);
  ec.setInputCloud(raw_hand);
  ec.extract(cluster_indices);
  
  if (cluster_indices.size() == 0)
    std::cout << "no cluster found and the data size before filtering is: " << raw_hand->size() << std::endl;
  else
  {
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
  raw_hand_filtered.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
     {
       raw_hand_filtered -> points.push_back(raw_hand->points[*pit]);
       RawHand_coord.push_back(temp1[*pit]);
    }
//   std::cout << raw_hand_filtered->size() << "points in raw hand cluster after filtering" << std::endl;
   std::cout << "There are " << RawHand_coord.size() << " points in raw hand after clustering" << std::endl;
  }
}

