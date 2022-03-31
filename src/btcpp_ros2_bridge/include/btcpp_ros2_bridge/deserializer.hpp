#ifndef DESERIALIZER_HPP
#define DESERIALIZER_HPP

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/flatbuffers/BT_logger_generated.h>
#include <behaviortree_cpp_v3/flatbuffers/bt_flatbuffer_helper.h>

#include "tree_node.hpp"

std::string NodeTypeToStr(BT::NodeType e) {
    switch (e) {
        case BT::NodeType::UNDEFINED:
        return "UNDEFINED";
        case BT::NodeType::ACTION:
        return "ACTION";
        case BT::NodeType::CONDITION:
        return "CONDITION";
        case BT::NodeType::CONTROL:
        return "CONTROL";
        case BT::NodeType::DECORATOR:
        return "DECORATOR";
        case BT::NodeType::SUBTREE:
        return "SUBTREE";
        default:
        return "UNKNOWN";
    }
}


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
    
    std::unordered_map<std::string, tree_node> nodes; // store the tree nodes in a map ordered by their unique "registration_ID"

    for( const Serialization::TreeNode* fb_node: *(fb_behavior_tree->nodes()) )
    {

        tree_node node;

        node.instance_name = fb_node->instance_name()->c_str();
        node.registration_ID = fb_node->registration_name()->c_str();
        node.status = convert(fb_node->status());
        node.uid = fb_node->uid();

        // iterate through child_uid and add to node.children_uid
        for(int child_uid: *(fb_node->children_uid()))
        {
            node.children_uid.push_back(child_uid);
        }

        nodes.insert( { node.registration_ID, node } );

    }

    for( const Serialization::NodeModel* model_node: *(fb_behavior_tree->node_models()) )
    {
        std::string registration_ID = model_node->registration_name()->c_str();
        BT::NodeType type = convert(model_node->type());

        nodes[registration_ID].type = type; 
    }

    std::unordered_map<uint16_t, tree_node> tree; // store the tree nodes in a map ordered by their uid

    for (auto& node: nodes)
    {
        tree.insert( { node.second.uid, node.second } );
    }

    return tree;

}

#endif // DESERIALIZER_HPP