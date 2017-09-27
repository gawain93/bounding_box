#ifndef DATACREATE_H
#define DATACREATE_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/concept_check.hpp>
#include <boost/graph/graph_concepts.hpp>

#include<opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <string.h>
#include <sstream>

class datacreate
{
public:
  datacreate(std::string &Video_name, int Middle_h, int Middle_w,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr Raw_hand,
	     pcl::PointCloud<pcl::PointXYZRGB>::Ptr Full_view,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr Bounding_hand,
	     int Frame_index);
//   ~datacreate();
  void mask_create();
  void center_find();
  
  cv::Mat mask;        // final result
  cv::Mat depth_map;
  cv::Mat rgb_map;
  cv::Mat rgb_map_compare;
  cv::Mat init_hand;                                                      // after distance threshold
  cv::Mat init_hand_clustered;                                            // after largest clustering
  cv::Mat after_kd;                                                       // after kd tree to remove the object
  cv::Point hand_center;              // 2D coordinate of hand center
  Eigen::Vector3f middle_point;
  int frame_index;
  cv::Vec2i middle_2d;
  int middle_x_2d, middle_y_2d;
  std::vector<cv::Point2i> middle_point_list;
  std::vector<int> middle_x_list; std::vector<int> middle_y_list;
private:
//   cv::FileStorage fs("coordinates_and_mask.yml", cv::FileStorage::WRITE);
  const int img_height = 480;
  const int img_width = 640;
  
  static const float fx = 525.0;                                   // camera parameters
  static const float fy = 525.0;
  static const float cx = 319.5;
  static const float cy = 239.5;
  std::string video_name;
  int middle_h ; int middle_w;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw_hand;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_view;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr bounding_hand;
};


#endif