#ifndef TREE_NODE_HPP
#define TREE_NODE_HPP

#include <behaviortree_cpp_v3/flatbuffers/BT_logger_generated.h>
#include <behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h>


struct tree_node {

    std::string instance_name;
    std::string registration_ID;
    uint16_t uid;

    BT::NodeStatus status;

    // std::vector<flatbuffers::Offset<Serialization::PortConfig>> port_remaps;

    std::vector<uint16_t>  children_uid;

};


#endif