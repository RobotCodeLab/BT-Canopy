#ifndef DESERIALIZER_H
#define DESERIALIZER_H

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/flatbuffers/BT_logger_generated.h>
#include <behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h>


BT::NodeStatus convert(Serialization::NodeStatus type);

#endif // DESERIALIZER_H