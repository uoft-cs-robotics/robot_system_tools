#include "communication/state_publisher.h"


StatePublisher::StatePublisher(std::string port) : socket(ctx, zmq::socket_type::pub) {
    socket.bind(port);
    port = port;
}

StatePublisher::StatePublisher(const StatePublisher& publisher) : socket(ctx, zmq::socket_type::pub) {
    port = publisher.port;
    socket.bind(port);
}

void StatePublisher::writeMessage(const std::vector<double>& data) {
    zmq::message_t message(data);
    socket.send(message, zmq::send_flags::dontwait);
}