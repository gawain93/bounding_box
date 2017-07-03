#ifndef BOUNDBOX_H
#define BOUNDBOX_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
//#include <rviz/interactive_object.h>
#include <message_filters/subscriber.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/foreach.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <iostream>
#include <fstream>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/video/tracking.hpp>             // headfile for kalman filter
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class boundbox
{
public:
  boundbox(std::string fid);
  void load_data(std::string& data_path);
  void bounding_box();
  void largest_cluster();
  void find_input_edge();
  void find_hand_center();
  bool find_in_hand_indices(int element);
  void add_bounding_box();
  
  int hand_coordinate_middle_h;                                    // frame index of the middle of the hand
  int hand_coordinate_middle_w;
  int edge_dis;
  bool first_hand_found;
  void position_correction();
  double dt;
  int q_index;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_current_frame;                          
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_previous_frame; 
  
  std::vector<size_t> frame_index;                                 // to store the index of the frame 480*640
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr interest_points;           // used to show the detected interest points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Hand_points;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr ROI;
  
  Eigen::Vector3f middle_point;
  Eigen::Vector3f corrected_middle_point;
  
  enum Input_Edge
  {
    u = 0,
    d = 1,
    l = 2,
    r = 3
  };
  Input_Edge ie;
  
  // The following are for the kalman filter
  // The state variables are x,y,z, v_x,v_y,v_z
  // The measurement variables are M_x, M_y, M_z
  // No control variables
  int StateSize, MeasureSize, ColtrolSize ;
  int type;
  cv::Mat* state;
  cv::Mat* measure;
  
private:
  std::string frame_id;
  float hand_size = 0.1;                              // distance from the center of hand to the upper part of hand
  float box_size = 0.2; 
  
  std::vector<size_t> hand_index;                                  // to store the index of the point that is hand 
  std::vector <size_t> frame_mask;                                 // look up table of if the pixel is hand or not, 1 and 0
  
  cv::KalmanFilter* KF;
    
  static const float depth_threshold = 0.03;                       //  Threshold for the depth differences
  static const int frame_height = 480;                             // size of the frame
  static const int frame_width = 640;
  
  static const float fx = 525.0;                                   // camera parameters
  static const float fy = 525.0;
  static const float cx = 319.5;
  static const float cy = 239.5;
};

#endif