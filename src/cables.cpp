#include <ros/ros.h>
#include <mavros_msgs/WaypointList.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/gnc_functions.hpp" // GNC API with all ROS functions

// Adiciona as bibliotecas do YOLO
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/String.h>

// Bibliotecas do MAVROS
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/LaserScan.h>

#include <cmath>


// Definindo constantes
const double MIN_DISTANCE = 0.3; // Distância mínima de obstáculo
const double TAKEOFF_HEIGHT = 2.0; // Altura de decolagem
const double CABLE_DETECTION_THRESHOLD = 0.8; // Threshold para detectar o cabo


class ImageConverter
{

    //VARIAVEIS
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    // Adiciona as variáveis de detecção do YOLO
    ros::Subscriber bbox_sub_;
    bool bbox_received_;
    std::vector<darknet_ros_msgs::BoundingBox> bbox_data_;

    // Variáveis do MAVROS
    ros::Subscriber state_sub_;
    ros::Publisher local_pos_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient land_client_;
    ros::ServiceClient set_mode_client_;

    // Variáveis de decolagem
    bool is_armed_;
    bool is_ready_to_fly_;

    // Variáveis de controle de colisão
    ros::Subscriber lidar_sub_;
    bool obstacle_detected_;

    // Variáveis de controle de waypoints
    enum class State {TAKEOFF, MOVE_TO_WAYPOINT1, SEARCH_CABLE, FOLLOW_CABLE, MOVE_TO_WAYPOINT2, RETURN_HOME, LAND};
    State current_state_;
    mavros_msgs::WaypointList waypoint_list_;
    int current_waypoint_index_;


    //FUNCS
    public:
        ImageConverter()
        : it_(nh_), bbox_received_(false), is_armed_(false), is_ready_to_fly_(false),
        current_state_(State::TAKEOFF), current_waypoint_index_(0), obstacle_detected_(false)
        {
            // Subscrive to input video feed and publish output video feed
            image_sub_ = it_.subscribe("/webcam/image_raw", 1, &ImageConverter::imageCb, this);
            image_pub_ = it_.advertise("/image_converter/output_video", 1);

            // Subscrive ao tópico do YOLO
            bbox_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes", 1, &ImageConverter::bboxCb, this);

            // Inicializa as variáveis do MAVROS
            state_sub_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &ImageConverter::stateCb, this);
            local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
            arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
            land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
            set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

            // Subscrive ao tópico do LIDAR
            lidar_sub_ = nh_.subscribe("/scan", 1, &ImageConverter::lidarCb, this);

            // Define os waypoints
            waypoint_list_sub_ = nh_.subscribe("/mavros/mission/waypoints", 1, &ImageConverter::waypointListCb, this);
        }


        //WAYPOINTS
        void waypointListCb(const mavros_msgs::WaypointList::ConstPtr& msg)
        {
            // Armazena a lista de waypoints recebida na variável da classe
            ROS_INFO("Received %d waypoints", msg->waypoints.size());
            waypoint_list_ = *msg;
        }


        //MODO DE VOO
        bool setMode(const std::string& mode){
            mavros_msgs::SetMode set_mode_srv;
            set_mode_srv.request.base_mode = 0;
            set_mode_srv.request.custom_mode = mode;

            if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent){
                ROS_INFO("Mode %s enabled", mode.c_str());
                return true;
            }
            else{
                ROS_ERROR("Failed to set mode %s", mode.c_str());
                return false;
            }
        }

        //armar o drone
        bool arm()
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

            if (!is_armed_ && arming_client_.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
                is_armed_ = true;
                return true;
            }
            else
            {
                ROS_ERROR("Failed to arm vehicle");
                return false;
            }
        }


        //Decolagem
        bool takeoff()
        {
            mavros_msgs::CommandTOL takeoff_cmd;
            takeoff_cmd.request.altitude = 3;
            takeoff_cmd.request.latitude = 0;
            takeoff_cmd.request.longitude = 0;
            takeoff_cmd.request.yaw = 0;

            if (!is_ready_to_fly_ && land_client_.call(takeoff_cmd) && takeoff_cmd.response.success)
            {
                ROS_INFO("Vehicle in air");
                is_ready_to_fly_ = true;
                return true;
            }
            else
            {
                ROS_ERROR("Failed to takeoff");
                return false;
            }
        }

        // Pousar o drone
        bool land()
        {
            mavros_msgs::CommandTOL land_cmd;
            land_cmd.request.altitude = 0;
            land_cmd.request.latitude = 0;
            land_cmd.request.longitude = 0;
            land_cmd.request.yaw = 0;

            if (land_client_.call(land_cmd) && land_cmd.response.success)
            {
                ROS_INFO("Vehicle landed");
                return true;
            }
            else
            {
                ROS_ERROR("Failed to land");
                return false;
            }
        }

        // Implementa a função para receber o estado do drone
        void stateCb(const mavros_msgs::State::ConstPtr& msg)
        {
            if (msg->armed != is_armed_)
            {
                ROS_INFO("Vehicle armed state changed: %d", msg->armed);
                is_armed_ = msg->armed;
            }
        }


        //VOO
        void run()
        {
            switch (current_state_){
            case State::TAKEOFF:
                // Decolar e esperar até que o drone esteja no ar
                if (!is_armed_)
                    arm();
                else if (!is_ready_to_fly_)
                    takeoff();

                else current_state_ = State::MOVE_TO_WAYPOINT1;
                break;

            case State::MOVE_TO_WAYPOINT1:
                // Ir para o Waypoint1
                if (reachedWaypoint(waypoint_list_.waypoints[current_waypoint_index_]))
                {
                    current_waypoint_index_++;
                    if (current_waypoint_index_ >= waypoint_list_.waypoints.size())
                        current_state_ = State::LAND;

                    else current_state_ = State::SEARCH_CABLE;

                }else moveToWaypoint(waypoint_list_.waypoints[current_waypoint_index_]);
                break;

            case State::SEARCH_CABLE:
                // Procurar pelo cabo usando a câmera e o YOLO
                if (bbox_received_)
                {
                    if (cableDetected(bbox_data_))
                        current_state_ = State::FOLLOW_CABLE;

                    else ROS_WARN("Cable not detected");
                    bbox_received_ = false;

                } else ROS_INFO("Searching for cable");
                break;

            case State::FOLLOW_CABLE:
                // Seguir o cabo até o Waypoint2
                if (obstacle_detected_)
                {
                    ROS_WARN("Obstacle detected, stopping cable following.");
                    current_state_ = State::MOVE_TO_WAYPOINT2;
                    break;
                }

                if (reachedWaypoint(waypoint_list_.waypoints[current_waypoint_index_]))
                {
                    current_waypoint_index_++;
                    if (current_waypoint_index_ >= waypoint_list_.waypoints.size())
                        current_state_ = State::RETURN_HOME;
                    else current_state_ = State::FOLLOW_CABLE;

                } else followCable();
                break;

            case State::MOVE_TO_WAYPOINT2:
                // Ir para o Waypoint2
                if (reachedWaypoint(waypoint_list_.waypoints[current_waypoint_index_]))
                {
                    current_waypoint_index_++;
                    if (current_waypoint_index_ >= waypoint_list_.waypoints.size())
                        current_state_ = State::RETURN_HOME;

                    else current_state_ = State::SEARCH_CABLE;

                } else moveToWaypoint(waypoint_list_.waypoints[current_waypoint_index_]);
                break;

            case State::RETURN_HOME:
                current_waypoint_index_ = 0;
                if (reachedWaypoint(waypoint_list_.waypoints[current_waypoint_index_]))
                    current_state_ = State::LAND;
                
                else moveToWaypoint(waypoint_list_.waypoints[current_waypoint_index_])
                break;
            
            case State::LAND:
                if(land()){
                    ROS_INFO("SHOTDOWN DRONE!");
                    ros::shutdown();
                } current_state_ = State::RETURN_HOME;

                else 
                break;

            case default:
                break;
            }
        }
}


//SEGUE O CABO
void followCable()
{
    // Define a velocidade máxima do drone durante a missão
    const double max_speed = 1.0; // m/s

    // Define a distância máxima em que o drone pode estar do cabo
    const double max_distance = 5.0; // metros

    // Define a velocidade linear e angular do drone
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;

    // Loop principal da missão
    while (ros::ok() && current_state_.connected && current_state_.armed && !obstacle_detected_)
    {
        // Verifica se o drone chegou ao Waypoint2
        if (current_waypoint_index_ == 2)
        {
            ROS_INFO("Drone reached Waypoint2!");
            break;
        }

        // Verifica se a detecção do cabo foi realizada com sucesso
        if (!bbox_received_)
        {
            ROS_WARN("Cable not detected! Returning to HOME.");
            current_state_ = State::RETURN_HOME;
            break;
        }

        // Verifica se o drone está próximo o suficiente do cabo
        if (bbox_data_[0].xmax - bbox_data_[0].xmin < 200)
        {
            ROS_INFO("Reached cable");
            current_waypoint_index_++;
            current_state_ = State::MOVE_TO_WAYPOINT2;
            return;
        }

        // Encontra o centro do cabo na imagem
        int center_x = (bbox_data_[0].xmin + bbox_data_[0].xmax) / 2;
        int center_y = (bbox_data_[0].ymin + bbox_data_[0].ymax) / 2;

        // Calcula a velocidade necessária para seguir o cabo
        double cable_center = (bbox_data_[0].xmax + bbox_data_[0].xmin) / 2;
        double error = cable_center - image_width_ / 2;
        double vel = std::min(std::abs(error) / image_width_ * max_speed, max_speed);

        // Verifica a direção do erro e ajusta a velocidade de acordo
        if (error > 0)
            cmd_vel_.linear.y = -vel;
        else
            cmd_vel_.linear.y = vel;

        // Publica o comando de velocidade
        cmd_vel_pub_.publish(cmd_vel_);


        // Calcula a distância do drone ao cabo
        double distance = calculateDistance(center_x, center_y);

        // Verifica se o drone está perto o suficiente do cabo
        if (distance < max_distance)
        {
            // Define a velocidade linear do drone como a velocidade máxima
            linear_velocity = max_speed;

            // Calcula o erro na direção do cabo
            double error = (center_x - image_width_ / 2) / (image_width_ / 2);

            // Define a velocidade angular do drone proporcional ao erro
            angular_velocity = -1.0 * max_speed * error;
        }
        else
        {
            // Define a velocidade linear do drone como metade da velocidade máxima
            linear_velocity = max_speed / 2.0;

            // Define a velocidade angular do drone como zero
            angular_velocity = 0.0;
        }


        // Verifica se o drone está perto do chão (cabo caido)
        if (current_pose_.pose.position.z < 0.5)
        {
            ROS_WARN("Drone too close to the ground! Returning to HOME.");
            current_state_ = State::RETURN_HOME;
            break;
        }

        // Verifica se o drone está muito longe do cabo
        if (distance > 2.0 * max_distance)
        {
            ROS_WARN("Drone too far from the cable! Returning to SEARCH_CABLE.");
            current_state_ = State::SEARCH_CABLE;
            break;
        }

        // Verifica se o cabo foi perdido
        double distance = std::sqrt(std::pow(bbox_data_[0].xmin - bbox_data_[0].xmax, 2) + std::pow(bbox_data_[0].ymin - bbox_data_[0].ymax, 2));

        if (distance > cable_lost_distance)
        {
            ROS_WARN("Cable lost");
            current_state_ = State::RETURN_HOME;
        }

        // Define a mensagem de posição do drone
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = current_pose_.pose.position.x + linear_velocity * cos(current_yaw_);
        pose.pose.position.y = current_pose_.pose.position.y + linear_velocity * sin(current_yaw_);
        pose.pose.position.z = current_pose_.pose.position.z;
        pose.pose.orientation = current_pose_.pose.orientation;

        // Publica a mensagem de posição do drone
        local_pos_pub_.publish(pose);

        // Define a mensagem de atitude do drone
        mavros_msgs::AttitudeTarget att;
        att.type_mask = mavros_msgs::AttitudeTarget

    }
}


// Verifica se há obstáculos próximos ao drone
void lidarCb(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    float range_min = scan_msg->range_min;
    float range_max = scan_msg->range_max;
    float angle_min = scan_msg->angle_min;
    float angle_max = scan_msg->angle_max;
    float angle_increment = scan_msg->angle_increment;

    // Define a distância de segurança
    const double safety_distance = 2.0;

    // Verifica se há obstáculos próximos
    for (int i = 0; i < scan_msg->ranges.size(); i++)
    {
        if (scan_msg->ranges[i] < safety_distance)
        {
            obstacle_detected_ = true;
            ROS_WARN("Obstacle detected: %f", scan_msg->ranges[i]);
            return;
        }

        obstacle_detected_ = false;
    }
}


// Callback para o estado atual do drone
void stateCb(const mavros_msgs::State::ConstPtr& msg){
    current_state_ = *msg;
}


// Callback para as bounding boxes do YOLO
void bboxCb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg)
{
    bbox_data_ = bbox_msg->bounding_boxes;
    bbox_received_ = true;
}


// Callback dos MAVROS state
void stateCb(const mavros_msgs::State::ConstPtr& state_msg)
{
    is_armed_ = state_msg->armed;
    if (state_msg->mode == "OFFBOARD")
        is_ready_to_fly_ = true;
    else
        is_ready_to_fly_ = false;
}



int main(int argc, char **argv)
{
    // Inicia o nó ROS e Cria o objeto NodeHand
    ros::init(argc, argv, "~");
    ros::NodeHandle nh_;


  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

    // Gira o loop de execução até que o nó seja encerrado
    ros::spin();

    return 0;
}