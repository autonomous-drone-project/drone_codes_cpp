/* INCOMPLETO AINDA.. ERRO NA HR DE COMPILAR.. */

#include "../include/gnc_functions.hpp" // GNC API with all ROS functions
#include <mavlink/common/mavlink.h> // MavLink 


// Função para lidar com as mensagens MAVLink recebidas
void handleMavlinkMessage(const mavlink_message_t &message, std::vector<gnc_api_waypoint> &waypointList) {
    switch (message.msgid) {
        case MAVLINK_MSG_ID_MISSION_ITEM: {
            mavlink_mission_item_t missionItem;
            mavlink_msg_mission_item_decode(&message, &missionItem);

            // Extrair as informações relevantes do waypoint, como x, y, z, psi
            float x = missionItem.x;
            float y = missionItem.y;
            float z = missionItem.z;
            float psi = missionItem.param4;

            // Armazenar o waypoint na lista
            gnc_api_waypoint waypoint;
            waypoint.x = x;
            waypoint.y = y;
            waypoint.z = z;
            waypoint.psi = psi;
            waypointList.push_back(waypoint);

            break;
        }
    }
}


// Função para configurar a comunicação com o Mission Planner
void setupCommunication(std::vector<Waypoint> &waypointList) {
    // Configurar a comunicação com o Mission Planner (exemplo usando conexão serial)
    // ...

    // Loop principal para receber e processar as mensagens MAVLink
    while (ros::ok()){
        // Receber a próxima mensagem MAVLink
        mavlink_message_t message;

        // Lidar com a mensagem recebida
        handleMavlinkMessage(message, waypointList);

        // Verificar se todos os waypoints foram recebidos
        if (waypointList.size() >= 6) {
            // Todos os waypoints foram recebidos, podemos prosseguir com o processamento
            break;
        }

        // Aguardar um curto período de tempo antes de receber a próxima mensagem
        usleep(1000);
    }

    // Imprimir os waypoints recebidos
    for (const auto& waypoint : waypointList) {
        std::cout << "Waypoint: x=" << waypoint.x << ", y=" << waypoint.y << ", z=" << waypoint.z << ", psi=" << waypoint.psi << std::endl;
    }
}

void get_waypoints()
{
    wait4connect();
    wait4start();
    initialize_local_frame();

    std::vector<gnc_api_waypoint> waypointList;
    setupCommunication(waypointList);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gnc_node");
    ros::NodeHandle gnc_node("~");

    init_publisher_subscriber(gnc_node);

    get_waypoints();

    return 0;
}
