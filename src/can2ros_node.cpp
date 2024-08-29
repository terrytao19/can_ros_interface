#include <ros/ros.h>
#include <can_ros_interface/VehicleCan.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

void setupCANInterface() {
    system("sudo ip link set can0 down");
    system("sudo ifconfig can0 txqueuelen 1000");
    system("sudo ip link set can0 up type can bitrate 500000");
    system("cansend can0 000#0100000000000000");
    ROS_INFO("Enabled RES CAN");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "can2ros");
    ros::NodeHandle nh;

    // Create ROS publisher
    ros::Publisher pub = nh.advertise<can_ros_interface::VehicleCan>("vehicle_can", 1000);

    // Setup CAN interface
    setupCANInterface();

    // Open CAN socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket");
        return 1;
    }

    struct ifreq ifr;
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        return 1;
    }

    // Set the socket to non-blocking mode
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    struct can_frame frame;

    ros::Rate loop_rate(100);  // Loop frequency of 100 Hz

    while (ros::ok()) {
        int nbytes = read(s, &frame, sizeof(struct can_frame));
        if (nbytes < 0) {
            if (errno != EAGAIN) {
                perror("Read");
            }
            usleep(10000); // Sleep for a short time to avoid busy waiting
            continue;
        }

        can_ros_interface::VehicleCan vehicle_can_msg;
        vehicle_can_msg.header.stamp = ros::Time::now();

        char res_hex[17];
        snprintf(res_hex, sizeof(res_hex), "%02X%02X%02X%02X%02X%02X%02X%02X",
                 frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                 frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

        std::string res_str(res_hex);

        if (res_str == "0000000000000000") {
            vehicle_can_msg.res_switch = false;
            vehicle_can_msg.res_button = false;
            vehicle_can_msg.res_enabled = false;
        } else if (res_str == "0100008000005A01") {
            vehicle_can_msg.res_switch = false;
            vehicle_can_msg.res_button = false;
            vehicle_can_msg.res_enabled = false;
        } else if (res_str == "0100008000006401") {
            vehicle_can_msg.res_switch = false;
            vehicle_can_msg.res_button = false;
            vehicle_can_msg.res_enabled = true;
        } else if (res_str == "0300008000006401") {
            vehicle_can_msg.res_switch = true;
            vehicle_can_msg.res_button = false;
            vehicle_can_msg.res_enabled = true;
        } else if (res_str == "0700008000006401") {
            vehicle_can_msg.res_switch = true;
            vehicle_can_msg.res_button = true;
            vehicle_can_msg.res_enabled = true;
        } else if (res_str == "0500008000006401") {
            vehicle_can_msg.res_switch = false;
            vehicle_can_msg.res_button = true;
            vehicle_can_msg.res_enabled = true;
        } else {
            vehicle_can_msg.res_switch = false;
            vehicle_can_msg.res_button = false;
            vehicle_can_msg.res_enabled = false;
        }

        pub.publish(vehicle_can_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(s);
    return 0;
}
