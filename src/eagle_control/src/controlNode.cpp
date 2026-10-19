#include "eagle_control/canInterface.hpp"
#include "eagle_control/PIDcontroller.hpp"
#include "eagle_control/eagle_task.h"
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <ctime>
#include <cstdint>
#include <chrono>
#include <functional>
#include <sstream>
#include <iomanip>

#include <rclcpp/rclcpp.hpp>
#include <linux/can.h>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>

typedef struct LastestPath {
    nav_msgs::msg::Path path;
    std::chrono::steady_clock::time_point age;
} LastestPath;

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
    const int MAX_PATH_AGE = configFile["control"]["MAX_PATH_AGE"].as<int>();
    const float MAX_ACCEL = configFile["control"]["MAX_ACCEL"].as<float>();
    const float MAX_DECEL = configFile["control"]["MAX_DECEL"].as<float>();
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

rviz_2d_overlay_msgs::msg::OverlayText getOverlay(
    float speed, float target, float PIDout, 
    float throttle, float brake
) {
    rviz_2d_overlay_msgs::msg::OverlayText msg;

    msg.action = 0;

    msg.width = 300;
    msg.height = 180;

    msg.horizontal_alignment = 0;
    msg.vertical_alignment = 0;
    msg.horizontal_distance = 20;
    msg.vertical_distance = 20;

    msg.text_size = 18.0f;
    msg.font = "Fira Code";
    msg.line_width = 2;

    msg.bg_color.r = 0.05f;
    msg.bg_color.g = 0.05f;
    msg.bg_color.b = 0.05f;
    msg.bg_color.a = 0.85f;

    msg.fg_color.r = 1.0f;
    msg.fg_color.g = 1.0f;
    msg.fg_color.b = 1.0f;
    msg.fg_color.a = 1.0f;

    std::ostringstream text;
    text << std::fixed << std::setprecision(2)
         << "CONTROL DEBUG" << std::endl
         << "Speed: " << speed << std::endl
         << "Target: " << target << std::endl
         << "PID: " << PIDout << std::endl
         << "Throttle: " << throttle << std::endl
         << "Brake: " << brake;

    msg.text = text.str();

    return msg;
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

            debugPub = create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
                "/control/debug",
                10
            );

            speedPub = create_publisher<std_msgs::msg::Float64>(
                "/control/speed",
                10
            );

            targetPub = create_publisher<std_msgs::msg::Float64>(
                "/control/target",
                10
            );

            throttlePub = create_publisher<std_msgs::msg::Float64>(
                "/control/throttle",
                10
            );

            brakePub = create_publisher<std_msgs::msg::Float64>(
                "/control/brake",
                10
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
            if (!msg->poses.empty()) {
                lastestPath.path = *msg;
                havePath = true;
                lastestPath.age = std::chrono::steady_clock::now();
            }
        }

        void mainLoop() {
            if (!havePath) return;

            if (std::chrono::duration<float>(std::chrono::steady_clock::now() - lastestPath.age).count()*1000 > config.MAX_PATH_AGE) {
                std::cout << "-------Path error (returned path is empty)-------" << std::endl;

                controlTimer->cancel();
                rclcpp::shutdown();
                return;

            } else if (lastestPath.path.poses.size() > 1) {
                geometry_msgs::msg::PoseStamped secondToLastPose = lastestPath.path.poses[(int) lastestPath.path.poses.size() - 2];
                distanceToStop = secondToLastPose.pose.position.x;
                if ( secondToLastPose.pose.position.z == 1) {stopDetected = true;}
            } else {
                distanceToStop = 0;
            }

            if (stopDetected && !stopRequested) {
                stopRequested = true; 
                pid.reset();
            }

            rviz_2d_overlay_msgs::msg::OverlayText debugOverlay;

            float speed;
            if (!getSpeed(&speed)) {
                std::cout << "-------Speed error (Speed sensor broke or smth)-------" << std::endl;

                controlTimer->cancel();
                rclcpp::shutdown();
                return;
            };

            float target;
            if (!stopRequested) {
                target = config.TARGET_SPEED;
            } else {
                target = std::sqrt(2 * config.MAX_DECEL * distanceToStop);
            }

            const float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - pt).count();

            const float PIDout = pid.getPIDoutput(target, speed, dt);

            const float throttle = PIDout > 0 ? PIDout : 0;
            const float brake = PIDout < 0 ? -PIDout : 0;

            sendCommand(throttle, brake, false);

            if (
                speed < config.STOP_THRESHOLD && 
                stopRequested &&
                lastestPath.path.poses.back().pose.position.z == 1
            ) {
                sendCommand(0, 0, true);
                std::cout << "-------Elapsed time: " << std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count() << "-------" << std::endl;
                controlTimer->cancel();
                rclcpp::shutdown();
                return;
            }

            pt = std::chrono::steady_clock::now();

            debugOverlay = getOverlay(speed, target, PIDout, throttle, brake);

            debugPub->publish(debugOverlay);
            
            std_msgs::msg::Float64 speedMsg;
            speedMsg.data = speed;
            speedPub->publish(speedMsg);

            std_msgs::msg::Float64 targetMsg;
            targetMsg.data = target;
            targetPub->publish(targetMsg);

            std_msgs::msg::Float64 throttleMsg;
            throttleMsg.data = throttle;
            throttlePub->publish(throttleMsg);

            std_msgs::msg::Float64 brakeMsg;
            brakeMsg.data = brake;
            brakePub->publish(brakeMsg);
        }

        rclcpp::TimerBase::SharedPtr controlTimer;

        std::chrono::steady_clock::time_point pt;
        std::chrono::steady_clock::time_point startTime;

        LastestPath lastestPath;
        bool havePath, stopRequested, stopDetected;

        float distanceToStop;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr debugPub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPub, targetPub, throttlePub, brakePub;

        PIDcontroller pid;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::shared_ptr<ControlNode> node = std::make_shared<ControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}