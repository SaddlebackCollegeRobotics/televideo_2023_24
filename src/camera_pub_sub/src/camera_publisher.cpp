#include <sstream>
#include <string.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("camera_publisher", options);

    node->declare_parameter("device_index", 0);
    node->declare_parameter("image_width", 1280);
    node->declare_parameter("image_height", 720);
    node->declare_parameter("camera_fps", 30);
    node->declare_parameter("compression_format", "MJPG");
    node->declare_parameter("base_topic", "camera/image");

    int device_index = node->get_parameter("device_index").as_int();
    int imageWidth = node->get_parameter("image_width").as_int();
    int imageHeight = node->get_parameter("image_height").as_int();
    int camera_fps = node->get_parameter("camera_fps").as_int();
    std::string compression_format = node->get_parameter("compression_format").as_string();
    std::string base_topic = node->get_parameter("base_topic").as_string();

    image_transport::ImageTransport transport(node);
    image_transport::Publisher publisher = transport.advertise(base_topic, 1);

    // GStreamer pipeline for capturing from the camera, used by OpenCV
    std::ostringstream gstreamer_api;
    gstreamer_api << "v4l2src device=/dev/video" << device_index << " ! "
        << "image/jpeg,width=" << imageWidth << ","
        << "height=" << imageHeight << ","
        << "framerate=" << camera_fps << "/1,"
        << "format=(string)" << compression_format << " ! "
        << "decodebin ! appsink";

    cv::VideoCapture videoCapture(gstreamer_api.str(), cv::CAP_GSTREAMER);

    if (!videoCapture.isOpened()) {
        return 1;
    }

    cv::Mat frame;
    std_msgs::msg::Header header;
    sensor_msgs::msg::Image::SharedPtr msg;

    // This is in Hz
    rclcpp::WallRate loop_rate(camera_fps);

    while (rclcpp::ok()) 
    {
        videoCapture >> frame;

        if (!frame.empty()) 
        {
            msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            publisher.publish(msg);
            cv::waitKey(1);
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}
