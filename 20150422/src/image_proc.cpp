#include "ros/ros.h"
#include "sensor_msgs/Image.h" // #include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

ros::Publisher debug_pub;

class ImageConverter
{
  ros::NodeHandle nh_; // place _ in the end of member variables
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_) { // initializer, same with it_ = nh_; call constroctor of NodeHandle
    image_sub_ = it_.subscribe("image", 1, &ImageConverter::imageCb, this); // 4th arg?
    image_pub_ = it_.advertise("debug_image", 1);
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter() { // destructor
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // careful about image_encoding
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(250, 250), 100, CV_RGB(255, 0, 0));

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

// void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
//   sensor_msgs::Image new_msg; // msg containing processed image
//   new_msg = msg;
//   debug_pub.publish(new_msg);
// }

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;

  // ros::init(argc, argv, "image_proc");
  // ros::NodeHandle nh;
  // ros::Subscriber sub = nh.subscribe("image", 1, imageCallback);
  // debug_pub = nh.advertise<sensor_msgs::Image>("debug_image", 1);
  // ros::spin();
  // ros::Rate loop_rate(10); // memo
  // return 0; // unnecessary

}
