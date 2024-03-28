#include <string.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg, const std::string & window_name)
{
            cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

            std::cout << "Received image" << std::endl;

            cv::imshow(window_name, frame);

            cv::waitKey(1);
}

int main(int argc, char ** argv)
{
    const int QUEUE_SIZE = 1;

    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("camera_subscriber", options);

    node->declare_parameter("window_name", "camera_view");
    node->declare_parameter("base_topic", "camera/image");

    /* Set this parameter to the desired image transport.
     The topic name for this node will automatically be resolved to the appropriate 
     base topic + image transport name.
     The publisher automatically advertises all available image transports.
     Use 'ros2 run image_transport list_transports' to list available transports.
    */
    node->declare_parameter("image_transport", "compressed");

    std::string window_name = node->get_parameter("window_name").as_string();
    std::string base_topic = node->get_parameter("base_topic").as_string();

    cv::namedWindow(window_name, cv::WINDOW_NORMAL);
    cv::startWindowThread();

    image_transport::ImageTransport transport(node);
    image_transport::Subscriber subscriber = transport.subscribe(base_topic, QUEUE_SIZE, std::bind(topic_callback, std::placeholders::_1, window_name));
        
    rclcpp::spin(node);

    cv::destroyWindow(window_name);
        
    return 0;
}
