#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/Image.h> // sensor_msgs::Image
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp> // cvtColor, ...
#include <opencv2/highgui/highgui.hpp> // imshow

const std::string WINDOW_NAME = "Image";

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  auto image = cv_bridge::toCvCopy(msg);
  cv::imshow(WINDOW_NAME, image->image);
  cv::waitKey(1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber_cpp");

  ros::NodeHandle handle;

  auto sample = ros::topic::waitForMessage<sensor_msgs::Image>("image", handle, ros::Duration(3));
  
  if (!sample) {
    ROS_ERROR("Could not recieve an image message");
    return 0;
  }

  ROS_INFO("Recieved an image sample");

  cv::namedWindow(WINDOW_NAME, cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();

  ros::Subscriber subscriber = handle.subscribe("image", 10, imageCallback); 

  ros::spin();

  cv::destroyWindow(WINDOW_NAME);

  return 0;
}