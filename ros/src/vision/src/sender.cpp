#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

int main (int argc, char ** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    std::string topic_name, fd;
    int fps;
    bool is_device;
    nh.getParam("topic_name", topic_name);
    nh.getParam("fd", fd);
    nh.getParam("fps", fps);
    nh.getParam("is_device", is_device);

    std::string publisher_name = "/video/" + topic_name;
    image_transport::Publisher publisher = it.advertise(publisher_name, 5);

    cv::VideoCapture source;
    if (is_device) {
        uint8_t video_index = (uint8_t) (fd.back() - '0');
        source = cv::VideoCapture(video_index);
    } else {
        source = cv::VideoCapture(fd);
    }

    if (!source.isOpened()) {
        ROS_ERROR("Failed to open device on %s", fd.c_str());
        return -1;
    }

    ROS_INFO("Opened camera on %s", fd.c_str());
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
