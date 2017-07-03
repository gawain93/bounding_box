#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>

#include "boundbox.h"
#include "trackingbox.h"

ros::Publisher pub_test_cloud;
ros::Publisher pub_saved_cloud;
ros::Publisher pub_hand_cloud;
ros::Publisher pub_ROI_cloud ;
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

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;

using namespace visualization_msgs;

int main(int argc, char** argv)
{
         
  ros::init (argc, argv, "test_use");
  ros::NodeHandle nh;
  rosbag::Bag bag;
  bag.open("/home/dhri-dz/catkin_ws/save/2017-06-26-21-26-31.bag", rosbag::bagmode::Read);
  
  pub_test_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/HandCloud", 1000); 
  pub_saved_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/savedCloud", 1000);
  pub_ROI_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/ROICloud", 1000);
  pub_mtrack = nh.advertise<sensor_msgs::PointCloud2>("dhri/TransformedModel", 1000); 
  marker_pub1 = nh.advertise<visualization_msgs::Marker>("corrected_marker", 1);
  marker_pub2 = nh.advertise<visualization_msgs::Marker>("marker", 1);
    	   
  boundbox b_box(frame_id);
  trackingbox tracker;
      std::cout << "ros init" << std::endl;
  
  sensor_msgs::PointCloud2::ConstPtr reference = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points");
  frame_id = (reference -> header.frame_id); 
  
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
	   b_box.dt = current_time-past_time;                   // the time stamp between two frames used in kalman filter
	   b_box.q_index = query_index++;
	   
	   if (b_box.first_hand_found == 1)
	   {
	   b_box.largest_cluster();
	   
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
	   
	   if (file.is_open())
               file << b_box.middle_point << '\n';
	   file.close();
	   }
	   
	   past_time = current_time;
	   std::cout << "The current frame is " << query_index << std::endl;
	   	   
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





