#include "Publisher.h"
#include "NodeHandle.h"

namespace ros
{
    Publisher::Publisher(NodeHandle& nh) : _nh{nh} {};
}