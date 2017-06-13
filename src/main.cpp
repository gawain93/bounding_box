#include "boundbox.h"

ros::Publisher pub_test_cloud;
ros::Publisher pub_saved_cloud;
ros::Publisher pub_hand_cloud;
ros::Publisher marker_pub ;

sensor_msgs::PointCloud2ConstPtr current_frame;                 
sensor_msgs::PointCloud2ConstPtr previous_frame;

sensor_msgs::PointCloud2 temp1;                                  // comntain the point cloud which need to be visualized
sensor_msgs::PointCloud2 temp2;
sensor_msgs::PointCloud2 temp3;

std::string frame_id;                                            // frame id in rviz used for display
int query_index = 0;

void add_marker(float x, float y, float z);
void add_hand_sphere(float x, float y, float z);
visualization_msgs::Marker marker;

std::ofstream file("./hand_points.txt");

int main(int argc, char** argv)
{
         
  ros::init (argc, argv, "test_use");
  ros::NodeHandle nh;
  rosbag::Bag bag;
  bag.open("/home/dhri-dz/catkin_ws/save/2017-06-06-15-39-25.bag", rosbag::bagmode::Read);
  
  pub_test_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/HandCloud", 1000); 
  pub_saved_cloud = nh.advertise<sensor_msgs::PointCloud2>("/dhri/savedCloud",1000);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
 
  boundbox b_box(frame_id);
  
  sensor_msgs::PointCloud2::ConstPtr reference = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points");
  frame_id = (reference -> header.frame_id); 
  
  rosbag::View view(bag, rosbag::TopicQuery("/camera/depth_registered/points"));
  BOOST_FOREACH(rosbag::MessageInstance const m, view)    // loop through view
    {
       current_frame = m.instantiate<sensor_msgs::PointCloud2>();   // return the pointer of the message
       if (current_frame != NULL)
       {
	 b_box.pcl_current_frame.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	 pcl::fromROSMsg(*current_frame,*b_box.pcl_current_frame);
	 b_box.frame_index.clear();
	 
	 // publish saved frame here
	 temp1 = *current_frame;
	 temp1.header.frame_id = frame_id.c_str();
	 pub_saved_cloud.publish(temp1);
	 
	 if (query_index == 0)
	 {
	    b_box.pcl_previous_frame.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
	    *b_box.pcl_previous_frame = *b_box.pcl_current_frame;
	    std::cout << query_index++ << std::endl;
	 }
	 else
	 {
	   b_box.bounding_box();
	   b_box.largest_cluster();
	   
	   pcl::toROSMsg(*b_box.Hand_points, temp2);          // publish extracted hand
           temp2.header.frame_id = frame_id.c_str();
           pub_test_cloud.publish(temp2);
	   
	   std::cout << "start to find edge" << std::endl; 
	   b_box.find_input_edge();	   
	   b_box.find_hand_center();
	   add_hand_sphere(b_box.middle_point(0),
                           b_box.middle_point(1),
	                   b_box.middle_point(2));
	   marker_pub.publish(marker);	   
	   
	   if (file.is_open())
               file << b_box.middle_point << '\n';
	   file.close();
	   
	   std::cout << query_index++ << std::endl;
	 }
	 
       }
     }
    
    bag.close();
     
return (0);
}  

void add_marker(float x, float y, float z)

{
  std::cout << "The marked coor is " << x << ' ' << y << ' ' << z << std::endl;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();                 // time_stamp
  marker.ns = "my_marker";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 0;
  
  marker.type = visualization_msgs::Marker::CUBE;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 1.0f;             // set color to red
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  
  marker.lifetime = ros::Duration();
}

void add_hand_sphere(float x, float y, float z)
{
  std::cout << "The marked coor is " << x << ' ' << y << ' ' << z << std::endl;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();                 // time_stamp
  marker.ns = "my_sphere";
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 0;
  
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 0.0f;             // set color to red
  marker.color.g = 0.0f;
  marker.color.b = 1.0f;
  marker.color.a = 0.2;
  
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  
  marker.lifetime = ros::Duration();

}

