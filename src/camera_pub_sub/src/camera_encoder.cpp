#include <sstream>  
#include <string.h>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "usb_device.h"


std::ostringstream gstreamer_api;
cv::VideoCapture videoCapture;
std::string camera_name;
std::string device_path;
std::string compression_format;

bool change_camera_resolution(int image_width, int image_height)
{
    // Get supported camera resolutions
    std::string command = "v4l2-ctl -d /dev/video0 --list-formats-ext";
    std::string resolution_list = exec(command.c_str());

    std::istringstream stream(resolution_list);
    std::string line;
    std::string endword = "]:";
    std::string keyword = "Size";
    std::string resolution_delim = "x";

    std::string device_path_found = "";
    size_t start_index = std::string::npos;

    while (std::getline(stream, line)) 
    {
        if (start_index == std::string::npos)
        {
            //Find start index
            start_index = line.find(compression_format);
        }
        else
        {
            // Get supported resolutions until end index
            
            size_t end_index = line.find(endword);

            if (end_index != std::string::npos)
                break;

            size_t resolution_index = line.find(keyword);

            if (resolution_index == std::string::npos)
                continue;

            size_t res_delim_index == line.find(resolution_delim);

            std::string left_side = line.substr(0, res_delim_index);
            std::string right_side = stoi(line.substr(res_delim_index + 1);
            // TODO
            

            







        }
    }


        std::string device_path = line.substr(0, delimiter_index);

        size_t keyword_index = device_path.find(keyword);

        // Get rid of device paths that are not video feeds
        if (keyword_index == std::string::npos)
            continue;

}

bool toggle_camera(bool enableCamera)
{   
    bool success = true;

    if (enableCamera)
    {
        if (!videoCapture.isOpened())
        {
            success = videoCapture.open(gstreamer_api.str(), cv::CAP_GSTREAMER);
        }
    }
    else
    {
        if (videoCapture.isOpened())
        {
            videoCapture.release();

            if (videoCapture.isOpened())
                success = false;
        }
        
    }

    if (!success)
    {
        std::cout << "ERROR! Unable to open camera: {name: "
        << camera_name << "}" << std::endl;
    }

    return success;
}

void toggle_camera_srv_process(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
          std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    response->success = toggle_camera(request->data);
}

void request_image_srv_process(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
        
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("camera_publisher", options);

    node->declare_parameter("camera_name", "camera1");
    node->declare_parameter("serial_ID", "");

    // Must be supported by physical camera device.
    node->declare_parameter("camera_cap_width", 1280);
    node->declare_parameter("camera_cap_height", 720);
    node->declare_parameter("camera_cap_fps", 30);

    // Ability to resize to something custom before sending.
    // These values should be equal to or less than camera_cap parameters.
    node->declare_parameter("image_send_width", 640);
    node->declare_parameter("image_send_height", 480);
    node->declare_parameter("image_send_fps", 5);

    node->declare_parameter("auto_enable_camera", false);
    node->declare_parameter("compression_format", "MJPG");

    // Currently implemented: amd64, jetson
    node->declare_parameter("host_machine", "jetson");

    int cameraCapWidth = node->get_parameter("camera_cap_width").as_int();
    int cameraCapHeight = node->get_parameter("camera_cap_height").as_int();
    int cameraCapFPS = node->get_parameter("camera_cap_fps").as_int();

    int imageSendWidth = node->get_parameter("image_send_width").as_int();
    int imageSendHeight = node->get_parameter("image_send_height").as_int();
    int imageSendFPS = node->get_parameter("image_send_fps").as_int();

    bool autoEnableCamera = node->get_parameter("auto_enable_camera").as_bool();

    std::string serial_ID = node->get_parameter("serial_ID").as_string();
    compression_format = node->get_parameter("compression_format").as_string();
    camera_name = node->get_parameter("camera_name").as_string();

    std::string hostMachine = node->get_parameter("host_machine").as_string();

    std::string base_topic = camera_name + "/transport";
    std::string toggle_srv_name = camera_name + "/toggle_camera";
    std::string request_image_srv_name = camera_name + "/request_image";

    // TODO - Switch to image_transport::CameraPublisher to get access to qos
    image_transport::ImageTransport transport(node);
    image_transport::Publisher publisher = transport.advertise(base_topic, 1);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_camera_srv = 
    node->create_service<std_srvs::srv::SetBool>(toggle_srv_name, &toggle_camera_srv_process);

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr request_image_srv = 
    node->create_service<std_srvs::srv::SetBool>(request_image_srv_name, &request_image_srv_process);

    device_path = get_device_path(serial_ID);

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
    if (hostMachine == "jetson")
    {
        gstreamer_api << "v4l2src device=" << device_path << " io-mode=2"<< " ! "
        << "image/jpeg,width=" << cameraCapWidth << ","
        << "height=" << cameraCapHeight << ","
        << "framerate=" << cameraCapFPS << "/1 ! "
        << "jpegdec" << " ! "
       	<< "video/x-raw ! appsink";
    }
    else if (hostMachine == "amd64")
    {
        gstreamer_api << "v4l2src device=" << device_path << " ! "
        << "image/jpeg,width=" << cameraCapWidth << ","
        << "height=" << cameraCapHeight << ","
        << "framerate=" << cameraCapFPS << "/1,"
        << "format=(string)" << compression_format << " ! "
        << "decodebin ! appsink";
    }

    if (autoEnableCamera)
    {
        toggle_camera(true);
    }

    cv::Mat frame;
    cv::Mat resizedFrame;
    std_msgs::msg::Header header;
    sensor_msgs::msg::Image::SharedPtr msg;

    // This is in Hz
    rclcpp::WallRate loop_rate(imageSendFPS);

    while (rclcpp::ok()) 
    {
        if (videoCapture.isOpened())
        {
            videoCapture >> frame;

            if (!frame.empty()) 
            {
                cv::resize(frame, resizedFrame, cv::Size(imageSendWidth, imageSendHeight), 0.0, 0.0, cv::INTER_AREA);
                msg = cv_bridge::CvImage(header, "bgr8", resizedFrame).toImageMsg();
                publisher.publish(msg);
            }
        }
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}
