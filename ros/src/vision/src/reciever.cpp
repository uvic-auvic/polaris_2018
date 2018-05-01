#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

std::string name;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
      cv::imshow(name, cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception e)
    {
      ROS_ERROR("Couldn't convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_reciever");
    ros::NodeHandle nh("~");


    // Open Window and setup image conversion
    nh.getParam("name", name);    
    cv::namedWindow(name);
    cv::startWindowThread();

    // Convert image from ROS image to regular image
    image_transport::ImageTransport it(nh);
    std::string subscribe_name = "/video/" + name;
    image_transport::Subscriber sub = it.subscribe(subscribe_name, 5, imageCallback);
    // Keep refreshing as images come in
    ros::spin();

    // Cleanup
    cv::destroyWindow(name);
    return 0;
}
