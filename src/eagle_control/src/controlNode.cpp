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

//Path Object with Age value 
typedef struct LatestPath {
    nav_msgs::msg::Path path;
    std::chrono::steady_clock::time_point age;
} LatestPath;

//NOTE - Config Init
YAML::Node configFile = YAML::LoadFile("/home/eagle/ws/config/config.yaml");

struct config {
    const unsigned int VEH_SPEED = configFile["can"]["VEH_SPEED"].as<unsigned int>();
    const unsigned int AS_CMD = configFile["can"]["AS_CMD"].as<unsigned int>();
    const float KP = configFile["pid"]["KP"].as<float>();
    const float KI = configFile["pid"]["KI"].as<float>();
    const float KD = configFile["pid"]["KD"].as<float>();
    const float MIN = configFile["pid"]["MIN"].as<float>();
    const float MAX = configFile["pid"]["MAX"].as<float>();
    const float STOP_THRESHOLD = configFile["control"]["STOP_THRESHOLD"].as<float>();
    const float TARGET_SPEED = configFile["control"]["TARGET_SPEED"].as<float>();
    const int CLOCK_MS = configFile["control"]["CLOCK_MS"].as<int>();
    const int MAX_CAN_TRIES = configFile["control"]["MAX_CAN_TRIES"].as<int>();
    const int MAX_PATH_AGE = configFile["control"]["MAX_PATH_AGE"].as<int>();
    const float MAX_DECEL = configFile["control"]["MAX_DECEL"].as<float>();
    const float MAX_OUTPUT_STEP = configFile["control"]["MAX_OUTPUT_STEP"].as<float>();
} config;

CanInterface can("vcan0");

//SECTION - Private Functions

//NOTE - GetSpeed Function
bool getSpeed(float *speed) {
    struct can_frame frame{};
    bool receivedSpeed = false;

    //Wait and scan the first <config.MAX_CAN_TRIES> to receive VECH_SPEED
    for (int o = 0; o < config.MAX_CAN_TRIES && receivedSpeed == false; o++) {
        can.receiveFrame(frame);
        if (frame.can_id == EAGLE_TASK_VEH_SPEED_FRAME_ID) receivedSpeed = true;
    }

    if (receivedSpeed) {
        struct eagle_task_veh_speed_t decoded{};

        //Decode the speed usin the generated code
        const int result = eagle_task_veh_speed_unpack(&decoded, frame.data, frame.can_dlc);

        if (result < 0) {
            return false;
        } else {
            //Assign the speed to the pointer given as an arg
            *speed = (float) eagle_task_veh_speed_vehicle_speed_decode(decoded.vehicle_speed);
            return true;
        }
    } else {
        return false;
    }
}

//NOTE - sendCommand Function
void sendCommand(float throttle, float brake, bool missionFinished) {
    struct eagle_task_as_cmd_t command{};

    //Encode throttle brake and mission_finished
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

    //Send the frame to vCAN
    can.sendFrame(frame);
}

//NOTE - getOverlay Function
rviz_2d_overlay_msgs::msg::OverlayText getOverlay(
    float speed, float target, float PIDout, 
    float throttle, float brake
) {
    //Generate the overlay shown in Rviz UI to debug
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

//NOTE - rateLimiter Function
void rateLimiter(float target, float *current, float maxStep) {
    float dS = target - *current;

    if (dS > maxStep) {*current += maxStep; return;}
    if (dS < -maxStep) {*current -= maxStep; return;}
    *current = target; return;
}

//!SECTION

//SECTION - ControlNode Main Class
class ControlNode : public rclcpp::Node {
    public:
        //NOTE - Constructor
        ControlNode() : Node("ControlNode"),
        pid(config.KP, config.KI, config.KD, config.MIN, config.MAX) { //Also build the pidController obj
            //Subscribe to Centerline Path
            pathSub = create_subscription<nav_msgs::msg::Path>(
                "/planning/centerline",
                10,
                std::bind(
                    &ControlNode::getPath,
                    this,
                    std::placeholders::_1
                )
            );

            //Create all the publishers for both Rosbag & plots and Debugging
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

            havePath = brakeStart = stopDetected = missionFinished = nearStop = false;

            currentOutput = 0;

            //Creation of the control timer to run mainLoop() every <config.CLOCK_MS> ms
            controlTimer = create_wall_timer(
                std::chrono::milliseconds(config.CLOCK_MS),
                std::bind(&ControlNode::mainLoop, this)
            );

            pt = startTime = std::chrono::steady_clock::now();
        }
    private: 
        //NOTE - getPath Function
        void getPath(const nav_msgs::msg::Path::SharedPtr msg) {
            //Everytime a new path "arrives", latestPath gets updated also with the time of arrival
            if (!msg->poses.empty()) {
                std::vector<geometry_msgs::msg::PoseStamped> stopPoints;
                std::copy_if( msg->poses.begin(), msg->poses.end(),
                    std::back_inserter(stopPoints),
                    [](const geometry_msgs::msg::PoseStamped& poseStamped) {
                        return poseStamped.pose.position.z == 1;
                    }
                );

                //As soon as u spot both points set nearStop to true
                if (stopPoints.size() > 1) {nearStop = true;}

                //If u spotted both stopping points once, if a path doesnt contain both now just discard it.
                if (!nearStop || stopPoints.size() > 1 || msg->poses.size() == 1) {
                    latestPath.path = *msg;
                    havePath = true;
                    latestPath.age = std::chrono::steady_clock::now();
                }
            }
        }

        //NOTE - mainLoop
        void mainLoop() {
            if (!havePath) return;

            //Mission Finished
            if (missionFinished ) {
                sendCommand(0, 0, true); 
                return;
            }

            //If a path becomes "too old", so no updates for more than <config.MAX_PATH_AGE>, the car stops
            if (std::chrono::duration<float>(std::chrono::steady_clock::now() - latestPath.age).count()*1000 > config.MAX_PATH_AGE) {
                std::cout << "-------Path error ( Path got older than: " << config.MAX_PATH_AGE << "ms )-------" << std::endl;

                controlTimer->cancel();
                rclcpp::shutdown();
                return;
            
            //Get the secondToLast pose, if the z value is 1.. The path contains the stopPoint
            } else if (latestPath.path.poses.size() > 1) {
                std::vector<geometry_msgs::msg::PoseStamped> stopPoints;
                std::copy_if( latestPath.path.poses.begin(), latestPath.path.poses.end(),
                    std::back_inserter(stopPoints),
                    [](const geometry_msgs::msg::PoseStamped& poseStamped) {
                    return poseStamped.pose.position.z == 1;
                });

                if (stopPoints.size() > 0) {
                    distanceToStop = stopPoints[0].pose.position.x;
                    stopDetected = true;
                }
            } else {
                distanceToStop = 0;
            }

            //Get current speed by passing the pointer.. 
            float speed;
            if (!getSpeed(&speed)) {//If getSpeed() returns false -> Bad Reading error or no speed at all
                std::cout << "-------Speed error (Speed sensor broke or smth)-------" << std::endl;

                controlTimer->cancel();
                rclcpp::shutdown();
                return;
            };

            //Get target speed
            float target;
            if (!stopDetected) { //<config.TARGET_SPEED> durin acceleration
                target = config.TARGET_SPEED;
            } else { //Formula to calculate fullStop distance
                target = std::sqrt(2 * config.MAX_DECEL * distanceToStop);
            }

            //Reset the PID before braking, mainly because of the Integral Error
            if (stopDetected && !brakeStart && target < speed) {
                brakeStart = true; 
                pid.reset();
            }

            //DeltaTime usin std::chrono
            const float dt = std::chrono::duration<float>(std::chrono::steady_clock::now() - pt).count();

            //Reading of PIDout and conversion to throttle and brake
            const float PIDout = pid.getPIDoutput(target, speed, dt);

            rateLimiter(PIDout, &currentOutput, config.MAX_OUTPUT_STEP);

            const float throttle = currentOutput > 0 ? currentOutput : 0;
            const float brake = currentOutput < 0 ? -currentOutput : 0;

            sendCommand(throttle, brake, false);

            if (
                speed < config.STOP_THRESHOLD && //Veichle goes close to 0m/s
                stopDetected && //Stop was requested
                latestPath.path.poses.back().pose.position.z == 1 && //Still the last 2 orange cones are visible
                latestPath.path.poses.size() == 1 //Car only sees the last cones, so its inside the box
            ) {
                missionFinished = true;

                std::cout << "-------Elapsed time: " << std::chrono::duration<float>(std::chrono::steady_clock::now() - startTime).count() << "-------" << std::endl;
            }

            //Publishers
            rviz_2d_overlay_msgs::msg::OverlayText debugOverlay;
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

            //Store previous loop time
            pt = std::chrono::steady_clock::now();
        }

        rclcpp::TimerBase::SharedPtr controlTimer;//Loop Control Timer

        std::chrono::steady_clock::time_point pt; //PreviousTime
        std::chrono::steady_clock::time_point startTime;

        LatestPath latestPath;
        bool havePath, brakeStart, stopDetected, missionFinished, nearStop;

        float distanceToStop, currentOutput;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSub;
        rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr debugPub;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPub, targetPub, throttlePub, brakePub;

        PIDcontroller pid; //pid object
};
//!SECTION

//NOTE - main Function
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::shared_ptr<ControlNode> node = std::make_shared<ControlNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}