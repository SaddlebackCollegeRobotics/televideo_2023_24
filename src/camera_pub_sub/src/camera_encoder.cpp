#include <sstream>  
#include <string.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include "usb_device.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("camera_publisher", options);

    node->declare_parameter("camera_name", "camera1");
    node->declare_parameter("serial_ID", "");
    node->declare_parameter("image_width", 1280);
    node->declare_parameter("image_height", 720);
    node->declare_parameter("camera_fps", 30);
    node->declare_parameter("compression_format", "MJPG");

    int imageWidth = node->get_parameter("image_width").as_int();
    int imageHeight = node->get_parameter("image_height").as_int();
    int camera_fps = node->get_parameter("camera_fps").as_int();

    std::string serial_ID = node->get_parameter("serial_ID").as_string();
    std::string compression_format = node->get_parameter("compression_format").as_string();
    std::string camera_name = node->get_parameter("camera_name").as_string();

    std::string base_topic = camera_name + "/transport";

    image_transport::ImageTransport transport(node);
    image_transport::Publisher publisher = transport.advertise(base_topic, 1);

    std::string device_path = get_device_path(serial_ID);

    if (device_path.empty()) {
        std::cout << "ERROR! Device not found: {name: "
        << camera_name << ", serial_ID: " 
        << serial_ID << "}" << std::endl;
        return 1;
    }
    else
    {
        std::cout << "Device found: {name: "
        << camera_name << ", device_path: " 
        << device_path << "}" << std::endl;
    }

    // GStreamer pipeline for capturing from the camera, used by OpenCV
    std::ostringstream gstreamer_api_amd64;
    gstreamer_api_amd64 << "v4l2src device=" << device_path << " ! "
        << "image/jpeg,width=" << imageWidth << ","
        << "height=" << imageHeight << ","
        << "framerate=" << camera_fps << "/1,"
        << "format=(string)" << compression_format << " ! "
        << "decodebin ! appsink";

    std::ostringstream gstreamer_api_jetson;
    gstreamer_api_jetson << "v4l2src device=" << device_path << " io-mode=2"<< " ! "
        << "image/jpeg,width=" << imageWidth << ","
        << "height=" << imageHeight << ","
        << "framerate=" << camera_fps << "/1 ! "
        << "jpegdec" << " ! "
       	<< "video/x-raw ! appsink";

    cv::VideoCapture videoCapture(gstreamer_api_jetson.str(), cv::CAP_GSTREAMER);

    if (!videoCapture.isOpened()) {
        std::cout << "ERROR! Unable to open camera: {name: "
        << camera_name << "}" << std::endl;
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
