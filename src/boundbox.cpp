#include "boundbox.h"

boundbox::boundbox(std::string fid)
{
  frame_id=fid;
}


void boundbox::bounding_box()
{
  int num_interest = 0;                          
  interest_points.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  frame_index.clear();
  
  for (size_t h = 0; h < frame_height; h++)
  {
    for (size_t w=0; w<frame_width; w++)
    {
      if (pcl_current_frame -> points[h*frame_width + w].z == NAN)
      {
	frame_index.push_back(0);
	continue;
      }
      if (pcl_previous_frame -> points[h*frame_width + w].z - pcl_current_frame -> points[h*frame_width + w].z >= depth_threshold)
      {
	frame_index.push_back(num_interest++);
	interest_points->points.push_back(pcl_current_frame -> points[h*frame_width + w]);
	continue;
      }
      frame_index.push_back(0);
    }
  }
  std::cout << "There are " << frame_index.size() << "points in total" << std::endl;
}


void boundbox::largest_cluster()
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree -> setInputCloud(interest_points);       
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(30000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(interest_points);          
  ec.extract(cluster_indices);
  
  hand_index.clear();
  if (cluster_indices.size() == 0)
    std::cout << "no cluster found" << std::endl;
  else
  {
  std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
  Hand_points.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
  {
       Hand_points -> points.push_back(interest_points->points[*pit]);      
       hand_index.push_back(*pit-1);
  }
  std::cout << std::endl  << Hand_points->size() << "points in hand cluster" << std::endl;
  
  frame_mask.clear();
  for (int h = 0; h < frame_height; h++)
  {
    for (int w = 0; w < frame_width; w++)
    {
      if (frame_index[h*frame_width + w] == 0)
	frame_mask.push_back(0);
      else
      {
	if (find_in_hand_indices(frame_index[h*frame_width + w]))
	  frame_mask.push_back(1);
	else
	  frame_mask.push_back(0);
      }
    }
  }
  
  }
}


void boundbox::find_input_edge()
{
  size_t up_dis = 0;  size_t down_dis = 0;
  size_t left_dis = 0; size_t right_dis = 0;
  std::vector<int> h1; std::vector<int> w1;
  std::vector<int> h2; std::vector<int> w2;
  std::vector<int> h3; std::vector<int> w3;
  std::vector<int> h4; std::vector<int> w4;
  
  std::vector<int> distance1;
  for (int w=0; w<frame_width; w++)           // loop for upper side
  {
    int first_none_nan = 0;
    for (int h=0; h<frame_height; h++)
    { 
      if (pcl_current_frame->points[h*frame_width+w].z == pcl_current_frame->points[h*frame_width+w].z)
      {
	first_none_nan = h;
	break;
      }
    }

    for (int h=0; h<frame_height; h++)
    {
      if (frame_mask[h*frame_width+w]==1)
      {
	distance1.push_back(h-first_none_nan);
	h1.push_back(h); w1.push_back(w);            // record all the points of the edge
	break;
      }
    }
  }
  up_dis = std::accumulate(distance1.begin(),distance1.end(),0.0)/float(distance1.size());
  
  std::vector<int> distance2;
  for (int w=0; w<640; w++)
  {
    int first_none_nan = 0;
    for (int h=480-1; h>=0; h--)
    { 
      if (pcl_current_frame->points[h*frame_width+w].z == pcl_current_frame->points[h*frame_width+w].z)
      {
	first_none_nan = h;
	break;
      }
    }
    for (int h = 480-1; h>=0; h--)
    {
      if (frame_mask[h*640+w]==1)
      {
	distance2.push_back(first_none_nan-h);
	h2.push_back(h); w2.push_back(w);
	break;
      }
    }
  }
  down_dis = std::accumulate(distance2.begin(),distance2.end(),0.0)/float(distance2.size());
  
  std::vector<int> distance3;
  for (int h = 0; h<480; h++)
  {
    int first_none_nan = 0;
    for (int w=0; w< 640; w++)
    { 
      if (pcl_current_frame->points[h*frame_width+w].z == pcl_current_frame->points[h*frame_width+w].z)
      {
	first_none_nan = w;
	break;
      }
    }
    for (int w=0; w< 640; w++)
    {
      if (frame_mask[h*640+w]==1)
      {
	distance3.push_back(w-first_none_nan);
	h3.push_back(h); w3.push_back(w);
	break;
      }
    }
  }
  left_dis = std::accumulate(distance3.begin(),distance3.end(),0.0)/float(distance3.size());
  
  std::vector<int> distance4;
  for (int h = 0; h<480; h++)
  {
    int first_none_nan = 0;
    for (int w=640-1; w>=0; w--)
    { 
      if (pcl_current_frame->points[h*frame_width+w].z == pcl_current_frame->points[h*frame_width+w].z)
      {
	first_none_nan = w;
	break;
       }
    }
    for (int w=640-1; w>=0; w--)
    {
      if (frame_mask[h*640+w]==1)
      {
	distance4.push_back(first_none_nan-w);
	h4.push_back(h); w4.push_back(w);
	break;
      }
    }
  }
  right_dis = std::accumulate(distance4.begin(),distance4.end(),0.0)/float(distance4.size());
  
  std::cout << "four distances are" << up_dis <<' '<< down_dis <<' '<< left_dis <<' '<< right_dis << std::endl;
  if (up_dis <= down_dis && up_dis <= left_dis && up_dis <= right_dis)
  {
    ie = u;edge_dis=up_dis;
    hand_coordinate_middle_h = h1[int(h1.size()/2)];
    hand_coordinate_middle_w = w1[int(w1.size()/2)];
  }
  if (down_dis <= up_dis && down_dis <= left_dis && down_dis <= right_dis)
  {
    ie = d;edge_dis=down_dis;
    hand_coordinate_middle_h = h2[int(h2.size()/2)];
    hand_coordinate_middle_w = w2[int(w2.size()/2)];
  }
  if (left_dis <= down_dis && left_dis <= up_dis && left_dis <= right_dis)
  {
    ie = l;edge_dis=left_dis;
    hand_coordinate_middle_h = h3[int(h3.size()/2)];
    hand_coordinate_middle_w = w3[int(w3.size()/2)];
  }
  if (right_dis <= down_dis && right_dis <= left_dis && right_dis <= up_dis)
  {
    ie = r;edge_dis=right_dis;
    hand_coordinate_middle_h = h4[int(h4.size()/2)];
    hand_coordinate_middle_w = w4[int(w4.size()/2)];
  }

}

void boundbox::find_hand_center()
{
  double dis_norm = -1000;
  double temp;
  int max_index;
  for (int i = 0; i<hand_index.size(); i++)
  {
    temp = (Hand_points->points[i].getVector3fMap()-pcl_current_frame
                ->points[hand_coordinate_middle_h*frame_width+hand_coordinate_middle_w].getVector3fMap()).norm();
    if (temp > dis_norm)
    {dis_norm = temp; max_index = i; }
  }
  
  /*
  std::cout << "The distance is " << dis_norm << std::endl;
  add_marker(Hand_points->points[max_index].x,
             Hand_points->points[max_index].y,
	     Hand_points->points[max_index].z);
  */
  
  Eigen::Vector3f e =( Hand_points -> points[max_index].getVector3fMap()-
                      pcl_current_frame->points[hand_coordinate_middle_h*frame_width+hand_coordinate_middle_w].getVector3fMap())/dis_norm;
  
  
  middle_point = Hand_points->points[max_index].getVector3fMap() - hand_size * e;  
  
  /*
  add_hand_sphere(middle_point(0),
                  middle_point(1),
	          middle_point(2));
  */	          
}


bool boundbox::find_in_hand_indices(int element)
{
  return (std::find(hand_index.begin(),hand_index.end(),element) != hand_index.end());
}
