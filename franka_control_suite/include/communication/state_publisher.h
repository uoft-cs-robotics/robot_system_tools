#pragma once

#include <zmq.hpp>
#include <vector>


class StatePublisher {
public:
    StatePublisher(std::string port);
    StatePublisher(const StatePublisher& publisher);
    void writeMessage(const std::vector<double>& data);

    zmq::context_t ctx;
    zmq::socket_t socket;
    std::string port;
};