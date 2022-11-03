#include <math.h>

#include "communication/action_subscriber.h"
#include <iostream>


ActionSubscriber::ActionSubscriber(CommsDataType dataType, std::string portId) : type(dataType), socket(ctx, zmq::socket_type::sub) {
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(portId);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = portId;
    action_space_dim_ = typeLengths[type];
    // values.push_back(0);
    // values.push_back(-M_PI_4);
    // values.push_back(0);
    // values.push_back(-3 * M_PI_4);
    // values.push_back(0);
    // values.push_back(M_PI_2);
    // values.push_back(M_PI_4);
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
}

ActionSubscriber::ActionSubscriber(const ActionSubscriber& subscriber) : type(subscriber.type), socket(ctx, zmq::socket_type::sub){
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(subscriber.port);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = subscriber.port;
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
}

void ActionSubscriber::readMessage() {
    zmq::message_t message;
    socket.recv(&message);
 
    int numValues = message.size() / sizeof(double);
    assert(numValues == action_space_dim_);
    
    std::lock_guard<std::mutex> guard(accessValuesMutex);
    for(int i = 0; i < numValues; i++)
        values[i] = *(reinterpret_cast<double*>(message.data()) + i);
}

void ActionSubscriber::readValues(std::vector<double>& output) {
    std::lock_guard<std::mutex> guard(accessValuesMutex);
    if(output.size() != values.size()) 
        output.resize(values.size());

    // std::cout << "received commands: ";
    for(size_t i = 0; i < values.size(); i++) {
        output[i] = values[i];
        // std::cout << output[i] << " ";
    }
    // std::cout << std::endl;

}

void ActionSubscriber::setDataType(CommsDataType dataType) {
    type = dataType;
    values.resize(typeLengths[dataType]);
    for(size_t i = 0; i < typeLengths[type]; i++)
        values[i] = 0.0;
}

double ActionSubscriber::readGripperCommands() {
    std::lock_guard<std::mutex> guard(accessValuesMutex);
    double finger1 = values.end()[-2];
    double finger2 = values.end()[-1];

    // std::cout << "finger 1: " << finger1 << " finger 2: " << finger2 << std::endl;

    return finger1 + finger2;
}