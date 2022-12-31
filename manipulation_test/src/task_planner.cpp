#include "manipulation_test/task_planner.hpp"

TaskPlanner::TaskPlanner(std::vector<unsigned int> number_of_intermediate_manifolds_, std::vector<unsigned int> number_of_manipulation_manifolds_)
{
    if(number_of_intermediate_manifolds_.size() != number_of_manipulation_manifolds_.size())
    {
        std::cout << "The number_of_intermediate_manifolds size should be equal to number_of_manipulation_manifolds size" << std::endl;
    }
    current_configuration_id = 0;
    current_action_node_id = 0;
    number_of_intermediate_manifolds = number_of_intermediate_manifolds_;
    number_of_manipulation_manifolds = number_of_manipulation_manifolds_;
    number_of_foliations = number_of_intermediate_manifolds_.size();

    manipulation_manifold_out_nodes.resize(number_of_foliations);
    manipulation_manifold_in_nodes.resize(number_of_foliations);

    intermediate_manifold_in_nodes.resize(number_of_foliations);
    intermediate_manifold_out_nodes.resize(number_of_foliations);

    for(int i = 0; i < number_of_foliations; i++)
    {
        manipulation_manifold_out_nodes[i].resize(number_of_manipulation_manifolds[i]);
        manipulation_manifold_in_nodes[i].resize(number_of_manipulation_manifolds[i]);

        intermediate_manifold_out_nodes[i].resize(number_of_intermediate_manifolds[i]);
        intermediate_manifold_in_nodes[i].resize(number_of_intermediate_manifolds[i]);
    }
}

// if the action is between two foliation, the motion trajectory should be start from last foliation and end at the next foliation.    
void TaskPlanner::addActionBetweenFoliations(moveit_msgs::RobotTrajectory motion_trajectory, unsigned int last_foliation_id, unsigned int last_manipulation_id, unsigned int next_foliation_id, unsigned int next_manipulation_id, float reward)
{
    // for each action between foliations, we generate only one node from the last foliation to the next foliation.
    TaskPlanner::ActionNode action_node;

    action_node.action_node_id = current_action_node_id;
    
    action_node.configuration_id = current_configuration_id;

    action_node.in_state.foliation_id = last_foliation_id;
    action_node.in_state.manifold_id = last_manipulation_id;
    action_node.in_state.is_in_manipulation_manifold = true;
    action_node.in_state.joint_values.clear();
    for(int i = 0; i < motion_trajectory.joint_trajectory.points[0].positions.size(); i++)
        action_node.in_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[0].positions[i]);

    action_node.out_state.foliation_id = next_foliation_id;
    action_node.out_state.manifold_id = next_manipulation_id;
    action_node.out_state.is_in_manipulation_manifold = true;
    action_node.out_state.joint_values.clear();
    for(int i = 0; i < motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions.size(); i++)
        action_node.out_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions[i]);

    action_node.in_state.value = 0.0;
    action_node.out_state.value = 0.0;

    action_node.isBetweenFoliations = true;

    action_node.motion_trajectory = motion_trajectory;

    action_node.reward = reward;

    action_nodes.push_back(action_node);

    manipulation_manifold_in_nodes[last_foliation_id][last_manipulation_id].push_back(current_action_node_id);

    current_configuration_id++;
    current_action_node_id++;
}

// if the action is between two manifolds, the motion trajectory should be start from intermediate manifold and end at the next manipulation manifold.
void TaskPlanner::addActionBetweenManifolds(moveit_msgs::RobotTrajectory motion_trajectory, unsigned int intermedaite_manifold_id, unsigned int manipulation_manifold_id, unsigned int foliation_id, float reward)
{
    // for each action between manifolds, we generate two nodes with opposite directions.
    TaskPlanner::ActionNode action_node1;
    TaskPlanner::ActionNode action_node2;

    action_node1.action_node_id = current_action_node_id;

    // action node 1 is from manipulation manifold to intermediate manifold
    action_node1.configuration_id = current_configuration_id;

    action_node1.in_state.foliation_id = foliation_id;
    action_node1.in_state.manifold_id = manipulation_manifold_id;
    action_node1.in_state.is_in_manipulation_manifold = true;
    action_node1.in_state.joint_values.clear();
    for(int i = 0; i < motion_trajectory.joint_trajectory.points[0].positions.size(); i++)
        action_node1.in_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[0].positions[i]);

    action_node1.out_state.foliation_id = foliation_id;
    action_node1.out_state.manifold_id = intermedaite_manifold_id;
    action_node1.out_state.is_in_manipulation_manifold = false;
    action_node1.out_state.joint_values.clear();
    for(int i = 0; i < motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions.size(); i++)
        action_node1.out_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions[i]);

    action_node1.in_state.value = 0.0;
    action_node1.out_state.value = 0.0;

    action_node1.isBetweenFoliations = false;

    action_node1.motion_trajectory = motion_trajectory;

    action_node1.reward = reward;

    action_nodes.push_back(action_node1);

    manipulation_manifold_in_nodes[foliation_id][manipulation_manifold_id].push_back(current_action_node_id);
    intermediate_manifold_out_nodes[foliation_id][intermedaite_manifold_id].push_back(current_action_node_id);

    current_action_node_id++;

    action_node2.action_node_id = current_action_node_id;

    // action node 2 is from intermediate manifold to manipulation manifold
    action_node2.configuration_id = current_configuration_id;

    action_node2.in_state.foliation_id = foliation_id;
    action_node2.in_state.manifold_id = intermedaite_manifold_id;
    action_node2.in_state.is_in_manipulation_manifold = false;
    action_node2.in_state.joint_values.clear();
    for(int i = 0; i < motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions.size(); i++)
        action_node2.in_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions[i]);

    action_node2.out_state.foliation_id = foliation_id;
    action_node2.out_state.manifold_id = manipulation_manifold_id;
    action_node2.out_state.is_in_manipulation_manifold = true;
    action_node2.out_state.joint_values.clear();
    for(int i = 0; i < motion_trajectory.joint_trajectory.points[0].positions.size(); i++)
        action_node2.out_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[0].positions[i]);

    action_node2.in_state.value = 0.0;
    action_node2.out_state.value = 0.0;

    action_node2.isBetweenFoliations = false;

    action_node2.motion_trajectory = motion_trajectory;
    
    // reverse the motion trajectory
    for(int i = 0; i < motion_trajectory.joint_trajectory.points.size(); i++)
    {
        for(int j = 0; j < motion_trajectory.joint_trajectory.points[i].positions.size(); j++)
        {
            action_node2.motion_trajectory.joint_trajectory.points[i].positions[j] = motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1 - i].positions[j];
        }
    }

    action_node2.reward = reward;

    action_nodes.push_back(action_node2);

    intermediate_manifold_in_nodes[foliation_id][intermedaite_manifold_id].push_back(current_action_node_id);
    manipulation_manifold_out_nodes[foliation_id][manipulation_manifold_id].push_back(current_action_node_id);

    current_action_node_id++;
    current_configuration_id++;
}

void TaskPlanner::constructMDPGraph()
{
    for(int i = 0; i < action_nodes.size(); i++)
    {
        // if the action node is from manipulation manifold to intermediate manifold
        if(action_nodes[i].isBetweenFoliations == false && action_nodes[i].in_state.is_in_manipulation_manifold == true)
        {
            action_nodes[i].next_action_node_ids.clear();
            for(unsigned int possible_action_node_id: intermediate_manifold_in_nodes[action_nodes[i].out_state.foliation_id][action_nodes[i].out_state.manifold_id])
            {
                if(action_nodes[possible_action_node_id].configuration_id != action_nodes[i].configuration_id)
                    action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
            }
            if(action_nodes[i].next_action_node_ids.size() > 0)
                action_nodes[i].policy = action_nodes[i].next_action_node_ids[0];
        } 
        else if(action_nodes[i].isBetweenFoliations == false && action_nodes[i].in_state.is_in_manipulation_manifold == false) // if the action node is from intermediate manifold to manipulation manifold
        {
            action_nodes[i].next_action_node_ids.clear();
            for(unsigned int possible_action_node_id: manipulation_manifold_in_nodes[action_nodes[i].out_state.foliation_id][action_nodes[i].out_state.manifold_id])
            {
                if(action_nodes[possible_action_node_id].configuration_id != action_nodes[i].configuration_id)
                    action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
            }
            if(action_nodes[i].next_action_node_ids.size() > 0)
                action_nodes[i].policy = action_nodes[i].next_action_node_ids[0];
        }
        else if(action_nodes[i].isBetweenFoliations == true) // if the action node is between foliations
        {
            // for now, we ignore the action nodes between foliations
            continue;
        }
    }
}

void TaskPlanner::policyIteration()
{


}

void TaskPlanner::policyEvaluation()
{

}