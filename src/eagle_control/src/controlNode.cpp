#include "eagle_control/canInterface.hpp"
#include "eagle_control/PIDcontroller.hpp"
#include "eagle_control/eagle_task.h"
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
    float STOP_THRESHOLD = configFile["control"]["STOP_THRESHOLD"].as<float>();
} config;

CanInterface can("vcan0");

float getSpeed() {
    struct can_frame frame{};

    do {
        can.receiveFrame(frame);
    } while (frame.can_id != EAGLE_TASK_VEH_SPEED_FRAME_ID);

    struct eagle_task_veh_speed_t decoded{};

    const int result = eagle_task_veh_speed_unpack(
        &decoded,
        frame.data,
        frame.can_dlc
    );

    if (result < 0) {
        return 0.0f;
    }

    return static_cast<float>(
        eagle_task_veh_speed_vehicle_speed_decode(
            decoded.vehicle_speed
        )
    );
}

void sendCommand(float throttle, float brake, bool missionFinished) {
    struct eagle_task_as_cmd_t command{};

    command.throttle_req = eagle_task_as_cmd_throttle_req_encode(throttle);

    command.brake_req = eagle_task_as_cmd_brake_req_encode(brake);

    command.mission_finished = missionFinished ? 1 : 0;

    struct can_frame frame{};
    frame.can_id = EAGLE_TASK_AS_CMD_FRAME_ID;
    frame.can_dlc = EAGLE_TASK_AS_CMD_LENGTH;

    const int result = eagle_task_as_cmd_pack(
        frame.data,
        &command,
        frame.can_dlc
    );

    if (result < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("ControlNode"), "Failed to encode AS_CMD CAN frame");
        return;
    }

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

            havePath = stopRequested = false;

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

            if (
                lastestPath.poses.empty() ||
                (lastestPath.poses.back().pose.position.z == 1 &&
                lastestPath.poses.back().pose.position.x < config.STOP_DISTANCE)
            ) { 
                stopRequested = true; 
                pid.reset();
            }

            float target = stopRequested ? 0 : 40;

            float speed = getSpeed();
            float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - pt).count();

            float PIDout = pid.getPIDoutput(target, speed, dt);

            float throttle = PIDout > 0 ? PIDout : 0;
            float brake = PIDout < 0 ? -PIDout : 0;

            sendCommand(throttle, brake, false);

            if (speed < config.STOP_THRESHOLD && stopRequested) {
                sendCommand(0, 0, true);
                std::cout << "-------Elapsed time: " << std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count() << "-------" << std::endl;
                controlTimer->cancel();
                rclcpp::shutdown();
            }

            pt = std::chrono::steady_clock::now();
        }

        rclcpp::TimerBase::SharedPtr controlTimer;

        std::chrono::steady_clock::time_point pt;
        std::chrono::steady_clock::time_point startTime;

        nav_msgs::msg::Path lastestPath;
        bool havePath, stopRequested;

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