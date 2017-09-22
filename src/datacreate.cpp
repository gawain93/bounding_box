#include "datacreate.h"
#include "boundbox.h"
#include "kdsearch.h"

// cv::FileStorage fs("coordinates_and_mask.yml", cv::FileStorage::WRITE);

datacreate::datacreate(std::string& Video_name, int Middle_h, int Middle_w,
			 pcl::PointCloud< pcl::PointXYZRGB >::Ptr Raw_hand,
			 pcl::PointCloud< pcl::PointXYZRGB >::Ptr Full_view, 
			 pcl::PointCloud< pcl::PointXYZRGB >::Ptr Bounding_hand,
		         int Frame_index)
{ 
  video_name.clear();
  raw_hand.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  full_view.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  bounding_hand.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  
  video_name = Video_name;
  *raw_hand = *Raw_hand;
  *full_view = *Full_view;
  *bounding_hand = *Bounding_hand;
//   *middle_point_list = *Middle_point_list;
  
  middle_h = Middle_h; middle_w = Middle_w;
  mask = cv::Mat::zeros(img_height, img_width, CV_32F);  // flipped
  depth_map = cv::Mat::zeros(img_height, img_width, CV_32F);
  rgb_map = cv::Mat(img_height, img_width, CV_32FC3);
  rgb_map_compare = cv::Mat(img_height, img_width, CV_32FC3);
  init_hand = cv::Mat::zeros(img_height, img_width, CV_32FC3);
  init_hand_clustered = cv::Mat::zeros(img_height, img_width, CV_32FC3);
  after_kd = cv::Mat::zeros(img_height, img_width, CV_32FC3);
  frame_index = Frame_index;
  
 
  std::cout << "initialize finished" << std::endl;
}


void datacreate::mask_create()
{
  std::ostringstream os ;    // used for index conversion
  os << frame_index;
//   cv::FileStorage fs("/home/dhri-dz/savedata/coordinates_and_mask.yml", cv::FileStorage::WRITE);
  
  // save the hand mask
  for (int i = 0; i < RawHand_coord.size(); i++)           // RawHand_coord it is
  {
   mask.at<float>(RawHand_coord[i].x, RawHand_coord[i].y) = 255;    // flipped
//    std::cout << "There are " << RawHand_coord.size() << " points in raw hand after clustering" << std::endl;
//    std::cout << "Raw hand is" << RawHand_coord[i].x << RawHand_coord[i].y << std::endl;
  }
  cv::imwrite("/home/dhri-dz/savedata/mask/mask_" + os.str() + ".jpg", mask);
  
  // ****************** create the full color images ***********************************
  for (int h = 0; h < img_height; h++)
  {
    for (int w = 0; w < img_width; w++)
    {
      if (full_view -> points[h*640 + w].z == full_view -> points[h*640 + w].z)
      {
	depth_map.at<float>(h,w) = full_view -> points[h*640 + w].z;
	// start to generatet the rgb frames
        rgb_map.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	rgb_map.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	rgb_map.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
	
	rgb_map_compare.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	rgb_map_compare.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	rgb_map_compare.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
	
	init_hand.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	init_hand.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	init_hand.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
	
	init_hand_clustered.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	init_hand_clustered.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	init_hand_clustered.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
	
	after_kd.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	after_kd.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	after_kd.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
      }
      else
      {
	depth_map.at<float>(h,w) = 0;
	
 	rgb_map.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
 	rgb_map.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
 	rgb_map.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	
	rgb_map_compare.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	rgb_map_compare.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	rgb_map_compare.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
	
	init_hand.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	init_hand.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	init_hand.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
	
	init_hand_clustered.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	init_hand_clustered.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	init_hand_clustered.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
	
	after_kd.at<Vector3f>(h,w)[2] = full_view -> points[h*640 + w].r;
	after_kd.at<Vector3f>(h,w)[1] = full_view -> points[h*640 + w].g;
	after_kd.at<Vector3f>(h,w)[0] = full_view -> points[h*640 + w].b;
      }
    }
  }
  
  /********* for debugging ***********************************************************************************/
   for (int j = 0; j < Test_coord.size(); j++)
   {
     init_hand.at<Vector3f>(Test_coord[j].x , Test_coord[j].y)[0] = 0;
     init_hand.at<Vector3f>(Test_coord[j].x , Test_coord[j].y)[1] = 0;
     init_hand.at<Vector3f>(Test_coord[j].x , Test_coord[j].y)[2] = 0;
  }
  cv::imwrite("/home/dhri-dz/savedata/test1/init_hand_" + os.str() + ".jpg", init_hand);
  
   for (int k = 0; k < after_bounding_box.size(); k++)
   {
     init_hand_clustered.at<Vector3f>(after_bounding_box[k].x , after_bounding_box[k].y)[0] = 0;
     init_hand_clustered.at<Vector3f>(after_bounding_box[k].x , after_bounding_box[k].y)[1] = 0;
     init_hand_clustered.at<Vector3f>(after_bounding_box[k].x , after_bounding_box[k].y)[2] = 0;
  }
  cv::imwrite("/home/dhri-dz/savedata/test2/init_hand_clustered_" + os.str() + ".jpg", init_hand_clustered);
  
  for (int m = 0; m < after_KDsearch.size(); m++) 
   {
     after_kd.at<Vector3f>(after_KDsearch[m].x , after_KDsearch[m].y)[0] = 0;
     after_kd.at<Vector3f>(after_KDsearch[m].x , after_KDsearch[m].y)[1] = 0;
     after_kd.at<Vector3f>(after_KDsearch[m].x , after_KDsearch[m].y)[2] = 0;
  }
  cv::imwrite("/home/dhri-dz/savedata/test3/after_kd_" + os.str() + ".jpg", after_kd);
  /***********************************************************************************************************/
  
  
//   fs << "handmask of frame" + os.str()  << mask;
  std::cout << "There are " << RawHand_coord.size() << " points in the mask" << std::endl;
    
   // To find the middle point of the hand  
  float distance = -1000;
  int max_index = 0;
  for (int i = 0; i < raw_hand -> size(); i++)
  {
   float temp = (raw_hand->points[i].getVector3fMap()-full_view
                ->points[middle_h*640+middle_w].getVector3fMap()).norm(); 
   if (temp > distance) { distance = temp; max_index = i;}
  }
  
  Eigen::Vector3f e =( raw_hand -> points[max_index].getVector3fMap()-
                      full_view->points[middle_h*640+middle_w].getVector3fMap())/distance;
		      
  middle_point = raw_hand->points[max_index].getVector3fMap() - 0.1 * e;  // hand size set as 0.2
  // To backproject the center point to the image coordinate !!!!!!!!!!!!!!!!
  middle_x_2d = round (fx * middle_point[0] / middle_point[2] + cx);              // round(fx / Hand_points->points[i].z * Hand_points -> points[i].x + cx);
  middle_y_2d = round (fy * middle_point[1] / middle_point[2] + cy);  
  middle_2d(0) = middle_x_2d; middle_2d(1) = middle_y_2d;
//   fs << "middle point of hand in 2D" + os.str() << middle_2d;
  middle_point_list.push_back(middle_2d);
  middle_x_list.push_back(middle_x_2d);  middle_y_list.push_back(middle_y_2d);
  
  // To generate the depth map
  cv::Mat depth_map_vis(depth_map);
  double min,max;
  cv::minMaxLoc(depth_map, &min, &max);
  // normalize the depthmap
  depth_map_vis = (depth_map / max) * 255;
  
 // ------------------- FOR DEBUGGING--------------------------------- 
  for (int i = 0; i < RawHand_coord.size(); i++)           // RawHand_coord it is
  {
//    depth_map_vis.at<int>(RawHand_coord[i].x, RawHand_coord[i].y) = 0;    // flipped
   rgb_map_compare.at<Vector3f>(RawHand_coord[i].x, RawHand_coord[i].y)[0] = 0;
   rgb_map_compare.at<Vector3f>(RawHand_coord[i].x, RawHand_coord[i].y)[1] = 0;
   rgb_map_compare.at<Vector3f>(RawHand_coord[i].x, RawHand_coord[i].y)[2] = 0;
  }
  //-------------------------------------------------------------------//
//   depth_map_vis.at<Vector3f>(middle_x_2d, middle_y_2d)[0] = 0;    // visulize the calculated middle point
//   depth_map_vis.at<Vector3f>(middle_x_2d, middle_y_2d)[1] = 255;
//   depth_map_vis.at<Vector3f>(middle_x_2d, middle_y_2d)[2] = 0;
 
  cv::imwrite("/home/dhri-dz/savedata/depthmap/depthmap_" + os.str() + ".jpg", depth_map_vis);
  cv::imwrite("/home/dhri-dz/savedata/rgbmap/rgbmap_" + os.str() + ".jpg", rgb_map);
  cv::imwrite("/home/dhri-dz/savedata/test/compare_" + os.str() + ".jpg", rgb_map_compare);
//   fs << "depth map" + os.str()  << depth_map;
 
}


 
