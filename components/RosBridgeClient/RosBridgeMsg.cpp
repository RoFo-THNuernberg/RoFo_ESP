#include "RosBridgeMsg.h"

namespace ros
{   
    Advertise::~Advertise() {}

    Advertise::Advertise(std::string op, std::string topic, std::string type) : 
        _json_buffer{"{\"op\":\"" + op + "\",\"topic\":\"" + topic + "\",\"type\":\"" + type + "\"}"} {}

    const char* Advertise::getJSON()
    {   
        return _json_buffer.c_str();
    }   
}