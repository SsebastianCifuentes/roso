#include <iostream>
#include <boost/asio.hpp>
#include <chrono> // Para medir el tiempo
#include "detection.pb.h" // Incluir el archivo generado por Protobuf

int main() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    // Parámetros de red (deben coincidir con los del emisor)
    std::string multicast_address = "239.255.0.1";
    int port = 5000;

    try {
        boost::asio::io_context io_context;

        // Crear un socket UDP y un endpoint para el grupo multicast
        boost::asio::ip::udp::endpoint listen_endpoint(boost::asio::ip::address::from_string("0.0.0.0"), port);
        boost::asio::ip::udp::socket socket(io_context);

        socket.open(listen_endpoint.protocol());
        socket.set_option(boost::asio::ip::udp::socket::reuse_address(true));
        socket.bind(listen_endpoint);

        // Unirse al grupo multicast
        boost::asio::ip::address multicast_address_ip = boost::asio::ip::address::from_string(multicast_address);
        socket.set_option(boost::asio::ip::multicast::join_group(multicast_address_ip));

        std::cout << "Escuchando en " << multicast_address << ":" << port << std::endl;

        // Variables para el conteo de mensajes y el cálculo de mensajes por segundo
        int message_count = 0;
        auto start_time = std::chrono::steady_clock::now();

        while (true) {
            char data[1024];
            boost::asio::ip::udp::endpoint sender_endpoint;
            size_t length = socket.receive_from(boost::asio::buffer(data), sender_endpoint);

            // Deserializar el mensaje Protobuf
            detection::DetectedObject obj;
            if (obj.ParseFromArray(data, length)) {
                // Incrementar el contador de mensajes
                message_count++;

                if (obj.type() == detection::DetectedObject::ROBOT) {
                    std::string team = obj.team() == 1 ? "Equipo Rojo" : obj.team() == 2 ? "Equipo Azul" : "Desconocido";
                    std::cout << "Robot detectado: ";
                    std::cout << "Equipo: " << team << ", ";
                    std::cout << "Jugador ID: " << obj.id() << ", ";
                    std::cout << "Posición: (" << obj.x() << ", " << obj.y() << "), ";
                    std::cout << "Ángulo: " << obj.angle() << std::endl;
                } else if (obj.type() == detection::DetectedObject::BALL) {
                    std::cout << "Pelota detectada: ";
                    std::cout << "Posición: (" << obj.x() << ", " << obj.y() << ")" << std::endl;
                }
            } else {
                std::cerr << "Error al deserializar el mensaje Protobuf" << std::endl;
            }

            // Calcular el tiempo transcurrido
            auto current_time = std::chrono::steady_clock::now();
            std::chrono::duration<double> elapsed_seconds = current_time - start_time;
            if (elapsed_seconds.count() >= 1.0) {
                // Mostrar mensajes por segundo
                double messages_per_second = message_count / elapsed_seconds.count();
                std::cout << "Mensajes por segundo: " << messages_per_second << std::endl;
                // Resetear contador y tiempo
                message_count = 0;
                start_time = current_time;
            }
        }
    } catch (std::exception& e) {
        std::cerr << "Excepción: " << e.what() << std::endl;
    }

    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
