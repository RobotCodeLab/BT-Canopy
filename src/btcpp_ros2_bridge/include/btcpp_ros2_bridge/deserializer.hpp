#ifndef DESERIALIZER_HPP
#define DESERIALIZER_HPP

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/flatbuffers/BT_logger_generated.h>
#include <behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h>

#include "tree_node.hpp"

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

BT::NodeType convert(Serialization::NodeType type)
{
    switch (type)
    {
    case Serialization::NodeType::ACTION:
        return BT::NodeType::ACTION;
    case Serialization::NodeType::DECORATOR:
        return BT::NodeType::DECORATOR;
    case Serialization::NodeType::CONTROL:
        return BT::NodeType::CONTROL;
    case Serialization::NodeType::CONDITION:
        return BT::NodeType::CONDITION;
    case Serialization::NodeType::SUBTREE:
        return BT::NodeType::SUBTREE;
    case Serialization::NodeType::UNDEFINED:
        return BT::NodeType::UNDEFINED;
    }
    return BT::NodeType::UNDEFINED;
}

BT::PortDirection convert(Serialization::PortDirection direction) // Port direction is how variables move in BT
{
    switch (direction)
    {
    case Serialization::PortDirection::INPUT :
        return BT::PortDirection::INPUT;
    case Serialization::PortDirection::OUTPUT:
        return BT::PortDirection::OUTPUT;
    case Serialization::PortDirection::INOUT:
        return BT::PortDirection::INOUT;
    }
    return BT::PortDirection::INOUT;
}

std::unordered_map<uint16_t, tree_node> BuildTreeFromFlatbuffers(const Serialization::BehaviorTree *fb_behavior_tree)
{
    
    std::unordered_map<uint16_t, tree_node> tree; // store the tree nodes in a map ordered by their uid

    // for( const Serialization::NodeModel* model_node: *(fb_behavior_tree->node_models()) )
    // {
    //     std::string registration_ID = model_node->registration_name()->c_str();
    //     BT::NodeType type = convert(model_node->type());
    //     std::cout << "registration_ID: " << registration_ID << std::endl;
    //     std::cout << "type: " << type << std::endl;

    //     const flatbuffers::Vector<flatbuffers::Offset<Serialization::PortModel>> *ports = model_node->ports();
    //     std::cout << "num ports: " << ports->size() << std::endl;

    //     for(const Serialization::PortModel* port: *ports)
    //     {
    //         std::string port_name = port->port_name()->c_str();
    //         std::string type_name = port->type_info()->c_str();
    //         std::string description = port->description()->c_str();
    //         std::cout << "port_name: " << port_name << std::endl;
    //         std::cout << "type_name: " << type_name << std::endl;
    //         std::cout << "description: " << description << std::endl;
    //     }
    //     std::cout << "" << std::endl;
    // }

    for( const Serialization::TreeNode* fb_node: *(fb_behavior_tree->nodes()) )
    {
        // std::string instance_name = fb_node->instance_name()->c_str();
        // std::string registration_ID = fb_node->registration_name()->c_str();
        // BT::NodeStatus status = convert(fb_node->status());

        // std::cout << "instance_name: " << instance_name << std::endl;
        // std::cout << "registration_ID: " << registration_ID << std::endl;
        // std::cout << "status: " << status << std::endl;

        // for( const Serialization::PortConfig* pair: *(fb_node->port_remaps()) )

            
        //     std::string port_config_name = pair->port_name()->c_str();
        //     std::string port_config_remap = pair->remap()->c_str();

        //     std::cout << "port_config_name: " << port_config_name << std::endl;
        //     std::cout << "port_config_remap: " << port_config_remap << std::endl;

        //     // abs_node.ports_mapping.insert( { QString(pair->port_name()->c_str()),
        //     //                                  QString(pair->remap()->c_str()) } );
        // }

        // int uid = fb_node->uid();
        // std::cout << "uid: " << uid << std::endl;
        // std::cout << " " << std::endl;

        tree_node node;

        node.instance_name = fb_node->instance_name()->c_str();
        node.registration_ID = fb_node->registration_name()->c_str();
        node.status = convert(fb_node->status());
        node.uid = fb_node->uid();
        // node.port_remaps = fb_node->port_remaps();

        // iterate through child_uid and add to node.children_uid
        for(int child_uid: *(fb_node->children_uid()))
        {
            node.children_uid.push_back(child_uid);
        }

        tree.insert( { node.uid, node } );

    }

    return tree;

}

#endif // DESERIALIZER_HPP