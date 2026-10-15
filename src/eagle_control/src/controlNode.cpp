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
    const unsigned int VEH_SPEED = configFile["can"]["VEH_SPEED"].as<unsigned int>();
    const unsigned int AS_CMD = configFile["can"]["AS_CMD"].as<unsigned int>();
    const float KP = configFile["pid"]["KP"].as<float>();
    const float KI = configFile["pid"]["KI"].as<float>();
    const float KD = configFile["pid"]["KD"].as<float>();
    const float MIN = configFile["pid"]["MIN"].as<float>();
    const float MAX = configFile["pid"]["MAX"].as<float>();
    const float STOP_DISTANCE = configFile["control"]["STOP_DISTANCE"].as<float>();
    const float STOP_THRESHOLD = configFile["control"]["STOP_THRESHOLD"].as<float>();
    const float TARGET_SPEED = configFile["control"]["TARGET_SPEED"].as<float>();
    const int CLOCK_MS = configFile["control"]["CLOCK_MS"].as<int>();
    const int MAX_CAN_TRIES = configFile["control"]["MAX_CAN_TRIES"].as<int>();
} config;

CanInterface can("vcan0");

bool getSpeed(float *speed) {
    struct can_frame frame{};
    bool receivedSpeed = false;

    for (int o = 0; o < config.MAX_CAN_TRIES && receivedSpeed == false; o++) {
        can.receiveFrame(frame);
        if (frame.can_id == EAGLE_TASK_VEH_SPEED_FRAME_ID) receivedSpeed = true;
    }

    if (receivedSpeed) {
        struct eagle_task_veh_speed_t decoded{};

        const int result = eagle_task_veh_speed_unpack(&decoded, frame.data, frame.can_dlc);

        if (result < 0) {
            return false;
        } else {
            *speed = (float) eagle_task_veh_speed_vehicle_speed_decode(decoded.vehicle_speed);
            return true;
        }
    } else {
        return false;
    }
}

void sendCommand(float throttle, float brake, bool missionFinished) {
    struct eagle_task_as_cmd_t command{};

    command.throttle_req = eagle_task_as_cmd_throttle_req_encode(throttle);

    command.brake_req = eagle_task_as_cmd_brake_req_encode(brake);

    command.mission_finished = missionFinished ? 1 : 0;

    struct can_frame frame{};
    frame.can_id = EAGLE_TASK_AS_CMD_FRAME_ID;
    frame.can_dlc = EAGLE_TASK_AS_CMD_LENGTH;

    const int result = eagle_task_as_cmd_pack(frame.data, &command, frame.can_dlc);

    if (result < 0) {
        std::cout << "Failed to encode AS_CMD CAN frame";
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

            havePath = stopRequested = stopDetected = false;

            controlTimer = create_wall_timer(
                std::chrono::milliseconds(config.CLOCK_MS),
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

            if (lastestPath.poses.empty()) {
                std::cout << "-------Path error (returned path is empty)-------" << std::endl;

                controlTimer->cancel();
                rclcpp::shutdown();

            } else if (lastestPath.poses.size() > 1) {
                geometry_msgs::msg::PoseStamped secondToLastPose = lastestPath.poses[(int) lastestPath.poses.size() - 2];

                if (
                    secondToLastPose.pose.position.z == 1 &&
                    secondToLastPose.pose.position.x < config.STOP_DISTANCE
                ) {stopDetected = true;}
            }

            if (stopDetected && !stopRequested) {
                stopRequested = true; 
                pid.reset();
            }

            const float target = stopRequested ? 0 : config.TARGET_SPEED;

            float speed;
            if (!getSpeed(&speed)) {
                std::cout << "-------Speed error (Speed sensor broke or smth)-------" << std::endl;

                controlTimer->cancel();
                rclcpp::shutdown();
            };

            const float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - pt).count();

            const float PIDout = pid.getPIDoutput(target, speed, dt);

            const float throttle = PIDout > 0 ? PIDout : 0;
            const float brake = PIDout < 0 ? -PIDout : 0;

            sendCommand(throttle, brake, false);

            if (
                speed < config.STOP_THRESHOLD && 
                stopRequested &&
                lastestPath.poses.back().pose.position.z == 1
            ) {
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
        bool havePath, stopRequested, stopDetected;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
        PIDcontroller pid;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::shared_ptr<ControlNode> node = std::make_shared<ControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}