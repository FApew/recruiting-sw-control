#include "canInterface.hpp"
#include "PIDcontroller.hpp"
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <ctime>
#include <cstdint>
#include <chrono>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <nav_msgs/msg/path.hpp>

YAML::Node configFile = YAML::LoadFile("/home/eagle/ws/config/config.yaml");

struct config {
    unsigned int VEH_SPEED = configFile["can"]["VEH_SPEED"].as<unsigned int>();
    unsigned int AS_CMD = configFile["can"]["AS_CMD"].as<unsigned int>();
    float KP = configFile["pid"]["KP"].as<float>();
    float KI = configFile["pid"]["KI"].as<float>();
    float KD = configFile["pid"]["KD"].as<float>();
    float MIN = configFile["pid"]["MIN"].as<float>();
    float MAX = configFile["pid"]["MAX"].as<float>();
    float STOP_DISTANCE = configFile["control"]["STOP_DISTANCE"].as<float>();
} config;

CanInterface can("vcan0");

float getSpeed() {
    struct can_frame frame{};
    do {
        can.receiveFrame(frame);
    } while (frame.can_id != config.VEH_SPEED);

    //Leggo e unisco i primi 2 byte, dove c'è la speed moltiplicata per 0.01
    float rawSpeed = static_cast<uint16_t>(frame.data[0]) | (static_cast<uint16_t>(frame.data[1]) << 8);

    return rawSpeed * 0.01;
}

void sendCommand(float throttle, float brake, bool missionFinished) {
    struct can_frame frame{};

    frame.can_id = config.AS_CMD;
    frame.can_dlc = 8;

    const uint16_t rawThrottle = static_cast<uint16_t>(throttle * 1000.0f);
    const uint16_t rawBrake = static_cast<uint16_t>(brake * 1000.0f);

    frame.data[0] = rawThrottle & 0xFF;
    frame.data[1] = (rawThrottle >> 8) & 0xFF;

    frame.data[2] = rawBrake & 0xFF;
    frame.data[3] = (rawBrake >> 8) & 0xFF;

    frame.data[4] = missionFinished ? 1 : 0;

    can.sendFrame(frame);
}

class ControlNode : public rclcpp::Node {
    public:
        ControlNode() : Node("ControlNode"),
        pid(config.KP, config.KI, config.KD, config.MIN, config.MAX) {
            pathSub = create_subscription<nav_msgs::msg::Path>(
                "/planning/centerline",
                10,
                std::bind(
                    &ControlNode::getPath,
                    this,
                    std::placeholders::_1
                )
            );

            havePath = false;

            controlTimer = create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&ControlNode::mainLoop, this)
            );

            pt = startTime = std::chrono::steady_clock::now();
        }
    private: 
        void getPath(const nav_msgs::msg::Path::SharedPtr msg) {
            lastestPath = *msg;
            havePath = true;
        }

        void mainLoop() {
            if (!havePath) return;

            float target;

            float speed = getSpeed();
            float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - pt).count();

            if ((lastestPath.poses.back().pose.position.z == 1 &&
            lastestPath.poses.back().pose.position.x < config.STOP_DISTANCE) ||
            lastestPath.poses.empty()) {

                if (speed == 0) {
                    sendCommand(0, 0, true);
                    std::cout << "-------Elapsed time: " << std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count() << "-------" << std::endl;
                    controlTimer->cancel();
                    rclcpp::shutdown();
                } else {
                    target = 0;
                }
            } else {
                target = 40;
            }

            float PIDout = pid.getPIDoutput(target, speed, dt);

            float throttle = PIDout > 0 ? PIDout : 0;
            float brake = PIDout < 0 ? -PIDout : 0;

            sendCommand(throttle, brake, false);

            pt = std::chrono::steady_clock::now();
        }

        rclcpp::TimerBase::SharedPtr controlTimer;

        std::chrono::steady_clock::time_point pt;
        std::chrono::steady_clock::time_point startTime;

        nav_msgs::msg::Path lastestPath;
        bool havePath;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
        PIDcontroller pid;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<ControlNode> node = std::make_shared<ControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}