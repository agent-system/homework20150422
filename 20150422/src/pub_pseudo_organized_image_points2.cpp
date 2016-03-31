#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "std_msgs/Header.h"
#include "pcl_conversions/pcl_conversions.h"

ros::Publisher pub_;
void image_callback(const sensor_msgs::Image::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_pointcloud2");
  ros::NodeHandle n;
  ros::Subscriber sub_=n.subscribe("image", 1000, image_callback);
  pub_=n.advertise<sensor_msgs::PointCloud2> ("output", 1000);
  std::cout<<"a"<<std::endl;
  ros::Rate r(1);
  ros::spin();
  return 0;
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud (new pcl::PointCloud<pcl::PointXYZ> );
  cloud->width=msg->width;
  cloud->height=msg->height;
  cloud->points.resize(msg->width* msg->height);
  cloud->is_dense=false;
  //  ROS_INFO("msg frame:%s", msg->header.frame_id.c_str());  //frame: head_camera

  //  ROS_INFO("msg stamp.sec:%d stamp.nsec:%d", msg->header.stamp.sec, msg->header.stamp.nsec);
  pcl_conversions::toPCL(msg->header, cloud->header);
  //  ROS_INFO("msg->height:%d msg->width:%d", msg->height, msg->width);
  for (size_t i=0; i< msg->height; i++){
    for (size_t j=0; j< msg->width; j++){
      pcl::PointXYZ tmppnt;
      tmppnt.x=j;
      tmppnt.y=i;
      tmppnt.z=100*(i+j);
      cloud->points[i* msg->width + j]=tmppnt;
    }
  }
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header=msg->header;
  pub_.publish(ros_cloud);
}

