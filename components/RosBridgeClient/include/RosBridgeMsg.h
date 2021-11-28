#pragma once

#include <string>
#include <cstring>

namespace ros
{
    class RosBridgeMsg
    {   
        public:
            virtual ~RosBridgeMsg() = default;
            virtual const char *getJSON() = 0;
    };

    class Advertise : public RosBridgeMsg
    {
        public:
            Advertise(const std::string op, const std::string topic, const std::string type);
            ~Advertise();

            const char *getJSON() override;
            

        private:
            const std::string _json_buffer;
    };
}