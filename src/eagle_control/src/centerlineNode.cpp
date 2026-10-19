#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <eagle_msgs/msg/cone_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>

YAML::Node configFile = YAML::LoadFile("/home/eagle/ws/config/config.yaml");

struct config {
    const float CONES_MAX_X_OFFSET = configFile["path"]["CONES_MAX_X_OFFSET"].as<float>();
} config;

class CenterlineNode : public rclcpp::Node {
    public:
        CenterlineNode() : Node("CenterlineNode") {
            coneSubscriber = create_subscription<eagle_msgs::msg::ConeArray>(
                "/perception/cones",
                10,
                std::bind(
                    &CenterlineNode::processCones,
                    this,
                    std::placeholders::_1
                )
            );

            pathPublisher = create_publisher<nav_msgs::msg::Path>(
                "/planning/centerline",
                10
            );
        }
    private:
        void processCones(
            const eagle_msgs::msg::ConeArray::SharedPtr msg
        ) {
            std::vector<eagle_msgs::msg::Cone> leftCones, rightCones, OrangeCones;

            nav_msgs::msg::Path path;
            path.header = msg->header;

            for (size_t i = 0; i < msg->cones.size(); i++) {
                const eagle_msgs::msg::Cone &cone = msg->cones[i];

                if (cone.color == eagle_msgs::msg::Cone::YELLOW) {
                    rightCones.push_back(cone);
                } else if (cone.color == eagle_msgs::msg::Cone::BLUE) {
                    leftCones.push_back(cone);
                } else if (cone.color == eagle_msgs::msg::Cone::ORANGE_BIG) {
                    OrangeCones.push_back(cone);
                }
            }
       
            bool pair = false;
            for (size_t i = 0; i < leftCones.size(); i++) {
                pair = false;

                for (size_t j = 0; j < rightCones.size() && !pair; j++) {
                    if (std::fabs(leftCones[i].position.x - rightCones[j].position.x) < config.CONES_MAX_X_OFFSET) {
                        geometry_msgs::msg::PoseStamped poseStamped;
                        poseStamped.header = msg->header;

                        poseStamped.pose.position.x = leftCones[i].position.x;
                        poseStamped.pose.position.y = (float) (leftCones[i].position.y + rightCones[j].position.y) / 2;
                        poseStamped.pose.position.z = 0;

                        poseStamped.pose.orientation.w = 1;

                        path.poses.push_back(poseStamped);

                        pair = true;
                    }
                }
            }

            if (OrangeCones.size() % 2 == 0) {
                for (size_t i = 0; i < OrangeCones.size(); i+=2) {
                    geometry_msgs::msg::PoseStamped poseStamped;
                    poseStamped.header = msg->header;

                    poseStamped.pose.position.x = (float) (OrangeCones[i].position.x + OrangeCones[i+1].position.x) / 2;
                    poseStamped.pose.position.y = (float) (OrangeCones[i].position.y + OrangeCones[i+1].position.y) / 2;
                    poseStamped.pose.position.z = 1;

                    poseStamped.pose.orientation.w = 1;

                    path.poses.push_back(poseStamped);
                }                
            }

            pathPublisher->publish(path);
        }

    rclcpp::Subscription<eagle_msgs::msg::ConeArray>::SharedPtr coneSubscriber;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    const std::shared_ptr<CenterlineNode> node = std::make_shared<CenterlineNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}