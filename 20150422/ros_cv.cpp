#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr, bin_ptr(new cv_bridge::CvImage);
    try
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    Mat hsl,img_bin;
    cvtColor(cv_ptr->image,hsl, CV_RGB2HLS); 
    bin_ptr->header = cv_ptr->header;
    bin_ptr->encoding=sensor_msgs::image_encodings::MONO8;

    double min_h=100, max_h=100, min_s=100, max_s=155, min_l=155, max_l=155;
    Scalar hsl_lower(min_h, min_s, min_l);
    Scalar hsl_upper(max_h, max_s, max_l);
    inRange(hsl, hsl_lower, hsl_upper, img_bin);
     bin_ptr->image=img_bin;
    //bin_ptr->image=hsl;
    cv::namedWindow("src", 0);
    cv::imshow("src", hsl);

    cv::imshow(OPENCV_WINDOW, img_bin);
    cv::waitKey(5);
    // Output modified video stream
    image_pub_.publish(bin_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
