#include "../include/gnc_functions.hpp" // GNC API with all ROS functions

void fight()
{
    // Define a posição x do ponto de destino
    double destX = 4;  // Coordenada x do ponto de destino
    double altitute = 1.7;

    set_speed(0.05); // Velocidade linear desejada em m/s

    takeoff(altitute);

    while (ros::ok())
    {
        ros::spinOnce();

        geometry_msgs::Point current_pos = get_current_location();
        
        // Verifica se chegou ao ponto de destino
        if (current_pos.x >= destX){
            // Todos os waypoints foram percorridos, realiza o pouso
            land();
            break;
        }

        set_destination(destX, 0, altitute, 0);
        ros::Rate(2.0).sleep();
    }
}


void  connection()
{
    // Espera pela conexão com o veículo
    wait4connect();

    // Espera pelo início do sistema
    wait4start();

    // Inicializa o frame de referência local
    initialize_local_frame();

    // Define e percorre os waypoints
    fight();
}


int main(int argc, char** argv)
{
    // Inicializa o nó ROS
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    // Inicializa os publicadores e os assinantes
    init_publisher_subscriber(gnc_node);

    // Executa a sequência de decolagem e waypoints
    connection();

    return 0;
}
