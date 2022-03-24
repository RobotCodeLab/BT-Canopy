#include "deserializer.h"

BT::NodeStatus convert(Serialization::NodeStatus type)
{
    switch (type)
    {
    case Serialization::NodeStatus::IDLE:
        return BT::NodeStatus::IDLE;
    case Serialization::NodeStatus::SUCCESS:
        return BT::NodeStatus::SUCCESS;
    case Serialization::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;
    case Serialization::NodeStatus::FAILURE:
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::IDLE;
}
