#include <vector>
#include <rclcpp/rclcpp.hpp> //ROS 2 C++ Library
#include <eagle_msgs/msg/cone_array.hpp> //Custom cone array Include
#include <nav_msgs/msg/path.hpp>

class CenterlineNode : public rclcpp::Node { //Class "CenterlineNode" with parent rclcpp
    public:
        CenterlineNode() : Node("CenterlineNode") { //Constructor
            coneSubscriber = create_subscription<eagle_msgs::msg::ConeArray>( //iSCRIVITI E CLICCA LA CAMPANELLINA con type di dato "ConeArray" ( qjello di msg insomma)
                "/perception/cones", //Nome del topic al quale iscriversi
                10, //Maz Queue
                std::bind(
                    &CenterlineNode::processCones, //Quale funzione ( come callback javascript passo la funziona non la eseguo ( uso & ))
                    this, //Eseguila nell'istanza che l'ha chiamata ( = a js )
                    std::placeholders::_1 //Usa quello che ti arriva dalla subscription
                )
            );

            pathPublisher = create_publisher<nav_msgs::msg::Path>(
                "/planning/centerline",
                10
            );
        }
    private:
        void processCones(
            const eagle_msgs::msg::ConeArray::SharedPtr msg //Argoento con il msg
        ) {
            //Vettori conententi oggetti di tipo Cone per dividerli dx e sx ( ODIO LA SINTASSI :) )
            std::vector<eagle_msgs::msg::Cone> leftCones, rightCones;

            //Array da sputare fuori ogni callback
            nav_msgs::msg::Path path;

            path.header = msg->header;

            //Creo gli array dx e sx per comodità ( di base visto che il circuito è dritto non serve neanche separare dx e sx ma si sa mai )
            for (size_t i = 0; i < msg->cones.size(); i++) {
                const eagle_msgs::msg::Cone &cone = msg->cones[i]; //Grabbo il cono i

                if (cone.color == eagle_msgs::msg::Cone::YELLOW) {
                    rightCones.push_back(cone);
                } else if (cone.color == eagle_msgs::msg::Cone::BLUE) {
                    leftCones.push_back(cone);
                }
            }
       
            for (size_t i = 0; i < leftCones.size(); i++) {
                if (i < rightCones.size()) { //Controllo che al i cono sx ci sia l' i cono dx se no BOOM

                    geometry_msgs::msg::PoseStamped poseStamped; //Documentazione stupenda
                    poseStamped.header = msg->header;

                    poseStamped.pose.position.x = (float) (leftCones[i].position.x + rightCones[i].position.x) / 2;
                    poseStamped.pose.position.y = (float) (leftCones[i].position.y + rightCones[i].position.y) / 2;
                    poseStamped.pose.position.z = 0;

                    poseStamped.pose.orientation.w = 1;

                    path.poses.push_back(poseStamped);
                }
            }

            //pubblico tutto
            pathPublisher->publish(path);
        }

    //Creando lo SmartPointer per recuperare e pubblicare il Cone Array
    rclcpp::Subscription<eagle_msgs::msg::ConeArray>::SharedPtr coneSubscriber;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv); //Va beh faccio partire ros2

    //Creo il puntatore che contiene il coso che ho creato prima
    std::shared_ptr<CenterlineNode> node = std::make_shared<CenterlineNode>();

    //Lo tengo attivo per ascoltare eventi nuovi ( La subscription di prima )
    //Lo butto dentro a un event loop insomma tipo addEventListener ma più figo
    rclcpp::spin(node);

    //Ferma tutto stacca
    rclcpp::shutdown();

    //Lui c'è sempre mai se ne va
    return 0;
}