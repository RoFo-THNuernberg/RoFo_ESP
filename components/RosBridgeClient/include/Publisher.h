#pragma once

namespace ros
{
    class NodeHandle;

    class Publisher
    {
        public:
            Publisher(NodeHandle& nh);

        private:
            NodeHandle& _nh;
    };
}