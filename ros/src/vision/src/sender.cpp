/***************************************************************
 * @file image_sender.cpp
 * @brief The node which provides a video source of some kind
 * @date February 2017
/***************************************************************/

/***************************************************************
 * Includes
/***************************************************************/
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>


/*************************************************************
 * Implementation [Image Sender]
/*************************************************************/
int main (int argc, char ** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    std::string topic_name, fd;
    int fps;
    nh.getParam("topic_name", topic_name);
    nh.getParam("fd", fd);
    nh.getParam("fps", fps);

    std::string publisher_name = "/video/" + topic_name;
    image_transport::Publisher publisher = it.advertise(publisher_name, 5);
    uint8_t video_index = (uint8_t) (fd.back() - '0');
    
    cv::VideoCapture source(video_index - '0');
    if (!source.isOpened()) {
        ROS_ERROR("Failed to open device on %s", fd.c_str());
        return -1;
    }

    ros::Rate loop_rate(fps);

    while(nh.ok())
    {
        cv::Mat frame;
        source >> frame;

        if (frame.empty()) {
            continue;
        }

        // Convert to ROS image and send out
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        publisher.publish(msg);

        // Spin and wait
        cv::waitKey(1);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
