#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

class ImageProcess
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageProcess()
    :it_(nh_)
  {
    image_pub_ = it_.advertise("output", 1);
    image_sub_ = it_.subscribe("input", 1, &ImageProcess::imageCb, this);
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    
    cv::Mat img_org;
    cv::Mat img_out;
    img_org = cv_ptr->image.clone();
    img_out = img_org.clone();
    
  uint b_sum,g_sum,r_sum;
  b_sum = 0; g_sum = 0; r_sum = 0;
  for(int i=0; i<img_out.rows; i++){
    b_sum = 0; g_sum = 0; r_sum = 0;
    for(int j=0; j<img_out.cols; j++){
      b_sum += img_out.data[img_out.channels() * i * img_out.cols + j * img_out.channels() + 0];
      g_sum += img_out.data[img_out.channels() * i * img_out.cols + j * img_out.channels() + 1];
      r_sum += img_out.data[img_out.channels() * i * img_out.cols + j * img_out.channels() + 2];
    }
    for(int j=0; j<img_out.cols; j++){
      img_out.data[img_out.channels() * i * img_out.cols + j * img_out.channels() + 0] = b_sum / img_out.cols;
      img_out.data[img_out.channels() * i * img_out.cols + j * img_out.channels() + 1] = g_sum / img_out.cols;
      img_out.data[img_out.channels() * i * img_out.cols + j * img_out.channels() + 2] = r_sum / img_out.cols;
    }
  }
  cv_ptr->image = img_out;
  image_pub_.publish(cv_ptr->toImageMsg());
  }
};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "mean_image");
  ImageProcess ip;
  ros::spin();
  return 0;
}
