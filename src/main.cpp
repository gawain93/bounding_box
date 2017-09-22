#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>

#include "boundbox.h"
#include "trackingbox.h"
#include "kdsearch.h"
#include "datacreate.h"

ros::Publisher pub_test_cloud;
ros::Publisher pub_saved_cloud;
ros::Publisher pub_hand_cloud;
ros::Publisher pub_ROI_cloud ;
ros::Publisher pub_raw_hand;
ros::Publisher pub_mtrack;
ros::Publisher marker_pub1 ;
ros::Publisher marker_pub2 ;

sensor_msgs::PointCloud2ConstPtr current_frame;                 
sensor_msgs::PointCloud2ConstPtr previous_frame;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr particle_scene;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_model;

sensor_msgs::PointCloud2 temp1;                                  // comntain the point cloud which need to be visualized
sensor_msgs::PointCloud2 temp2;
sensor_msgs::PointCloud2 temp3;

std::string frame_id;                                            // frame id in rviz used for display
int query_index = 0;
double current_time = 0;
double past_time = 0;
bool manual_mode = 1;

void add_marker(float x, float y, float z);
void add_hand_sphere(float x, float y, float z);
visualization_msgs::Marker marker1;
visualization_msgs::Marker marker2;

std::ofstream file("./hand_points.txt");
std::vector<cv::Point2i> middle_point_list;
std::vector<int> middle_point_x_list; std::vector<int> middle_point_y_list;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

using namespace visualization_msgs;

int main(int argc, char** argv)
{
         
  ros::init (argc, argv, "test_use"); 
  ros::NodeHandle nh;
  rosbag::Bag bag;
  bag.open("/home/dhri-dz/catkin_ws/save/2017-09-05-13-13-32_new.bag", rosbag::bagmode::Read);  
  rosbag::Bag bag_save;
  bag_save.open("/home/dhri-dz/catkin_ws/save/save_data.bag", rosbag::bagmode::Write);
  std::string  video_name = "2017-09-05-13-13-32_new.bag";  
  
  pub_test_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/HandCloud", 1000); 
  pub_saved_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/savedCloud", 1000);
  pub_ROI_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/ROICloud", 1000);
  pub_mtrack = nh.advertise<sensor_msgs::PointCloud2>("dhri/TransformedModel", 1000);   
  pub_raw_hand = nh.advertise<sensor_msgs::PointCloud2>("dhri/RawHand", 1000); 
  marker_pub1 = nh.advertise<visualization_msgs::Marker>("corrected_marker", 1);
  marker_pub2 = nh.advertise<visualization_msgs::Marker>("marker", 1);
    	   
  boundbox b_box(frame_id);
  trackingbox tracker;
  
  sensor_msgs::PointCloud2::ConstPtr reference = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points");
  frame_id = (reference -> header.frame_id); 
  middle_point_list.clear();

  tracker.setModel();

  transformed_model = tracker.getTransformedModel();
        

  rosbag::View view(bag, rosbag::TopicQuery("/camera/depth_registered/points"));
  BOOST_FOREACH(rosbag::MessageInstance const m, view)    // loop through view
    {
       current_frame = m.instantiate<sensor_msgs::PointCloud2>();   // return the pointer of the message
       current_time = m.getTime().toSec();
       if (current_frame != NULL)
       {
	 b_box.pcl_current_frame.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	 pcl::fromROSMsg(*current_frame,*b_box.pcl_current_frame);
	 std::cout << "data loaded!" << std::endl;
	 particle_scene.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	 pcl::fromROSMsg(*current_frame, *particle_scene); 
	 tracker.run(particle_scene);
	 
	 sensor_msgs::PointCloud2 tran_model;
	 transformed_model = tracker.getTransformedModel();
	 pcl::toROSMsg(*transformed_model, tran_model);
	 tran_model.header.frame_id = frame_id;
	 pub_mtrack.publish(tran_model);
	 b_box.frame_index.clear();
	 
	 // publish saved frame here
	 temp1 = *current_frame;
	 temp1.header.frame_id = frame_id.c_str();
	 pub_saved_cloud.publish(temp1);
	 
	 if (query_index == 0)
	 {
	    b_box.pcl_previous_frame.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	    *b_box.pcl_previous_frame = *b_box.pcl_current_frame;
	    past_time = current_time;
	    std::cout << "The current frame is " << query_index++ << std::endl;
	 }
	 else
	 {	      
	   std::cout << "current time is" << current_time-past_time << std::endl;
	   b_box.bounding_box(); 
	   b_box.dt = current_time-past_time;                  // the time stamp between two frames used in kalman filter
	   b_box.q_index = query_index;       // ++ 
	   
	   if (b_box.first_hand_found == 1)  
	   {
	   b_box.largest_cluster();  
	   if (b_box.first_hand_found == 0)
	     continue;
	   
	   pcl::toROSMsg(*b_box.Hand_points, temp2);          // publish extracted hand
           temp2.header.frame_id = frame_id.c_str();
           pub_test_cloud.publish(temp2);
	   
	   std::cout << "start to find edge" << std::endl; 
	   b_box.find_input_edge();	   
	   b_box.find_hand_center();
	   b_box.add_bounding_box();
	   
	   pcl::toROSMsg(*b_box.ROI, temp3);          // publish extracted hand
           temp3.header.frame_id = frame_id.c_str();
           pub_ROI_cloud.publish(temp3);
	   
	   std::cout << "The size of ROI is:" << b_box.ROI -> size() << std::endl;
	   add_hand_sphere(b_box.corrected_middle_point(0),
                           b_box.corrected_middle_point(1),
	                   b_box.corrected_middle_point(2));
	   marker_pub1.publish(marker1);	   
	   
	   add_marker(b_box.middle_point(0),
                      b_box.middle_point(1),
	              b_box.middle_point(2));
	   marker_pub2.publish(marker2);	 
	   
	   // here to add the code to add the rode for bounding_box
	   
// 	   if (file.is_open())
//                file << b_box.middle_point << '\n';
// 	   file.close();
 	   }
	   
	   past_time = current_time;
	   std::cout << "The current frame is " << query_index << std::endl;
	  
	   
           kdsearch kd_search(b_box.ROI, transformed_model, tracker.model_tran_matrix);
 	   kd_search.search();
 	   kd_search.largest_cluster_filter();
 	   sensor_msgs::PointCloud2 raw_hand;
	   pcl::toROSMsg(*kd_search.raw_hand_filtered, raw_hand);
 	   raw_hand.header.frame_id = frame_id;
 	   pub_raw_hand.publish(raw_hand);
// 	   
// 	   bag_save.write("raw_hand", ros::Time::now(), raw_hand);
	   std::cout << "Start to create datasets" << std::endl;
           datacreate datacreater(video_name, b_box.hand_coordinate_middle_h, b_box.hand_coordinate_middle_w,
	                          kd_search.raw_hand_filtered, b_box.pcl_current_frame,b_box.Hand_points, query_index);
           datacreater.mask_create();	
	   std::ostringstream os ;    // used for index conversion
           os << query_index++;
	   cv::FileStorage fs("/home/dhri-dz/savedata/coordinates_and_mask.yml", cv::FileStorage::WRITE);
	   middle_point_list.push_back(datacreater.middle_2d);
	   middle_point_x_list.push_back(datacreater.middle_x_2d);
	   middle_point_y_list.push_back(datacreater.middle_y_2d);
	   
	   std::cout << "Hand middle point is" << datacreater.middle_2d[0] << ' ' << datacreater.middle_2d[1] << std::endl;
	   fs << "Number of frames processed" << os.str();
	   fs << "Middle point of hand "  << middle_point_list;
	   fs << "Middle point of hand x"  << middle_point_x_list;
	   fs << "Middle point of hand y"  << middle_point_y_list;
 	 }
       }
       //ros::spin();
       //server.reset();
     }

    bag.close();
    
return (0);
}  

void add_marker(float x, float y, float z)
{
  std::cout << "The marked coor is " << x << ' ' << y << ' ' << z << std::endl;
  marker2.header.frame_id = frame_id;
  marker2.header.stamp = ros::Time::now();                 // time_stamp
  marker2.ns = "my_marker";
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 0;
  
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.scale.x = 0.2;
  marker2.scale.y = 0.2;
  marker2.scale.z = 0.2;
  marker2.color.r = 1.0f;             // set color to red
  marker2.color.g = 0.0f;
  marker2.color.b = 0.0f;
  marker2.color.a = 0.2;
  
  marker2.pose.position.x = x;
  marker2.pose.position.y = y;
  marker2.pose.position.z = z;
  
  marker2.lifetime = ros::Duration();
}

void add_hand_sphere(float x, float y, float z)
{
  std::cout << "The marked coor is " << x << ' ' << y << ' ' << z << std::endl;
  marker1.header.frame_id = frame_id;
  marker1.header.stamp = ros::Time::now();                 // time_stamp
  marker1.ns = "my_sphere";
  marker1.action = visualization_msgs::Marker::ADD;
  marker1.pose.orientation.x = 0.0;
  marker1.pose.orientation.y = 0.0;
  marker1.pose.orientation.z = 0.0;
  marker1.pose.orientation.w = 0;
  
  marker1.type = visualization_msgs::Marker::SPHERE;
  marker1.scale.x = 0.2;
  marker1.scale.y = 0.2;
  marker1.scale.z = 0.2;
  marker1.color.r = 0.0f;             // set color to red
  marker1.color.g = 0.0f;
  marker1.color.b = 1.0f;
  marker1.color.a = 0.2;
  
  marker1.pose.position.x = x;
  marker1.pose.position.y = y;
  marker1.pose.position.z = z;
  
  marker1.lifetime = ros::Duration();

}





