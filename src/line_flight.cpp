#include <gnc_functions.hpp> // GNC API with all ROS functions

void fight()
{
    // Define a posição x do ponto de destino
    double destX = 1;  // Coordenada x do ponto de destino

    set_speed(0.3); // Velocidade linear desejada em m/s

    takeoff(1);

    while (ros::ok())
    {
        ros::spinOnce();

        // Verifica se chegou ao ponto de destino
        if (get_current_position().x >= destX){
            // Todos os waypoints foram percorridos, realiza o pouso
            land();
            break;
        }

        set_destination(destX, 0, 1, 0);
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
