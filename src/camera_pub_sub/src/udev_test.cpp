#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>
#include <cstring>


// may throw ament_index_cpp::PackageNotFoundError exception

// From https://stackoverflow.com/questions/478898/how-do-i-execute-a-command-and-get-the-output-of-the-command-within-c-using-po

std::string exec(const char* cmd) 
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);

    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }

    return result;
}

std::string get_usb_devices() {

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_pub_sub");
    std::string script_path = package_share_directory + "/resources/find_devpath.bash";

    // std::ostringstream oss;
    // oss << "#!/bin/bash\n"
    //     << "for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do\n"
    //     << "(\n"
    //     << "syspath=\"${sysdevpath%/dev}\"\n"
    //     << "devname=\"$(udevadm info -q name -p $syspath)\"\n"
    //     << "[[ \"$devname\" == \"bus/\"* ]] && exit\n"
    //     << "eval \"$(udevadm info -q property --export -p $syspath)\"\n"
    //     << "[[ -z \"$ID_SERIAL\" ]] && exit\n"
    //     << "echo \"/dev/$devname - $ID_SERIAL\"\n"
    //     << ")\n"
    //     << "done\n";

    // return exec(oss.str().c_str());

    std::string command = "chmod +x " + script_path;
    
    system(command.c_str());
    
    return exec(script_path.c_str());
}

int main() {
    std::string output = get_usb_devices();

    // TODO - Split output by new line.
    // Then split each line by " - " to get the device name and serial number.
    // Then compare with the serial number given by a ROS parameter.
    // If number matches, grab the device path.

    std::istringstream stream(output);
    std::string line;
    std::string delimiter = " - ";


    while (std::getline(stream, line)) {
        std::cout << line << std::endl;
        
    }

    // std::cout << "Result:\n" << output << std::endl;
    return 0;
}