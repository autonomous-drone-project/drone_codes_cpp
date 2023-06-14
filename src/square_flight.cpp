#include "../include/gnc_functions.hpp" // GNC API with all ROS functions

void set_waypoints()
{
    // Define a lista de waypoints
    std::vector<gnc_api_waypoint> waypointList = {
        // x, y, z, psi
        {0, 0, 2, 0},      // Waypoint 1: x=0, y=0, z=2, psi=0
        {2, 0, 2, -90},    // Waypoint 2: x=2, y=0, z=2, psi=-90
        {2, 2, 2, 0},      // Waypoint 3: x=2, y=2, z=2, psi=0
        {0, 2, 2, 90},     // Waypoint 4: x=0, y=2, z=2, psi=90
        {0, 0, 2, 180},    // Waypoint 5: x=0, y=0, z=2, psi=180
        {0, 0, 2, 0}       // Waypoint 6: x=0, y=0, z=2, psi=0
    };

    int counter = 0;
    while (ros::ok())
    {
        ros::spinOnce();

        // Verifica se o waypoint atual foi alcançado
        if (check_waypoint_reached(0.3) == 1)
        {
            if (counter < waypointList.size())
            {
                // Define o próximo waypoint como destino
                set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
                counter++;
            }
            else
            {
                // Todos os waypoints foram percorridos, realiza o pouso
                land();
                break;
            }
        }

        ros::Rate(2.0).sleep();
    }
}

void takeoff_and_wp()
{
    // Espera pela conexão com o veículo
    wait4connect();

    // Espera pelo início do sistema
    wait4start();

    // Inicializa o frame de referência local
    initialize_local_frame();

    // Realiza a decolagem com uma altitude de 3 metros
    takeoff(2);

    // Define e percorre os waypoints
    set_waypoints();
}

int main(int argc, char** argv)
{
    // Inicializa o nó ROS
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    // Inicializa os publicadores e os assinantes
    init_publisher_subscriber(gnc_node);

    // Executa a sequência de decolagem e waypoints
    takeoff_and_wp();

    return 0;
}
