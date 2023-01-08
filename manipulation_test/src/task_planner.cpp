#include "manipulation_test/task_planner.hpp"

TaskPlanner::TaskPlanner(std::vector<long unsigned int> number_of_intermediate_manifolds_, std::vector<long unsigned int> number_of_manipulation_manifolds_)
{

    discount_factor = 0.8;
    max_error = 0.0001;

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

    init_action_node_ids.clear();

    for(int i = 0; i < number_of_foliations; i++)
    {
        manipulation_manifold_out_nodes[i].resize(number_of_manipulation_manifolds[i]);
        manipulation_manifold_in_nodes[i].resize(number_of_manipulation_manifolds[i]);

        intermediate_manifold_out_nodes[i].resize(number_of_intermediate_manifolds[i]);
        intermediate_manifold_in_nodes[i].resize(number_of_intermediate_manifolds[i]);
    }
}

// if the action is between two foliation, the motion trajectory should be start from last foliation and end at the next foliation.    
void TaskPlanner::addActionBetweenFoliations(moveit_msgs::RobotTrajectory motion_trajectory, 
                                             long unsigned int last_foliation_id, long unsigned int last_manipulation_id, 
                                             long unsigned int next_foliation_id, long unsigned int next_manipulation_id, 
                                             float reward)
{
    // for each action between foliations, we generate only one node from the last foliation to the next foliation.
    TaskPlanner::ActionNode action_node;

    action_node.action_node_id = current_action_node_id;
    
    action_node.configuration_id = current_configuration_id;

    action_node.in_state.foliation_id = last_foliation_id;
    action_node.in_state.manifold_id = last_manipulation_id;
    action_node.in_state.is_in_manipulation_manifold = true;
    action_node.in_state.joint_values.clear();
    if(motion_trajectory.joint_trajectory.points.size() > 0)
    {
        for(int i = 0; i < motion_trajectory.joint_trajectory.points[0].positions.size(); i++)
            action_node.in_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[0].positions[i]);
    }

    action_node.out_state.foliation_id = next_foliation_id;
    action_node.out_state.manifold_id = next_manipulation_id;
    action_node.out_state.is_in_manipulation_manifold = true;
    action_node.out_state.joint_values.clear();
    if(motion_trajectory.joint_trajectory.points.size() > 0)
    {
        for(int i = 0; i < motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions.size(); i++)
            action_node.out_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions[i]);
    }

    action_node.in_state.value = 0.0;
    action_node.out_state.value = 0.0;

    action_node.isBetweenFoliations = true;

    action_node.motion_trajectory = motion_trajectory;

    action_node.reward = reward;

    action_nodes.push_back(action_node);

    manipulation_manifold_in_nodes[last_foliation_id][last_manipulation_id].push_back(current_action_node_id);
    manipulation_manifold_out_nodes[next_foliation_id][next_manipulation_id].push_back(current_action_node_id);

    current_configuration_id++;
    current_action_node_id++;
}

// if the action is between two manifolds, the motion trajectory should be start from intermediate manifold and end at the next manipulation manifold.
void TaskPlanner::addActionBetweenManifolds(moveit_msgs::RobotTrajectory motion_trajectory, 
                                            long unsigned int intermedaite_manifold_id, long unsigned int manipulation_manifold_id, 
                                            long unsigned int foliation_id, bool is_init_action, float reward)
{
    // for each action between manifolds, we generate two nodes with opposite directions.
    TaskPlanner::ActionNode action_node1;
    TaskPlanner::ActionNode action_node2;

    // action node 1 is from manipulation manifold to intermediate manifold
    action_node1.action_node_id = current_action_node_id;
    action_node1.configuration_id = current_configuration_id;

    action_node1.in_state.foliation_id = foliation_id;
    action_node1.in_state.manifold_id = manipulation_manifold_id;
    action_node1.in_state.is_in_manipulation_manifold = true;
    action_node1.in_state.joint_values.clear();
    if(motion_trajectory.joint_trajectory.points.size() > 0)
    {
        for(int i = 0; i < motion_trajectory.joint_trajectory.points[0].positions.size(); i++)
            action_node1.in_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[0].positions[i]);
    }   

    action_node1.out_state.foliation_id = foliation_id;
    action_node1.out_state.manifold_id = intermedaite_manifold_id;
    action_node1.out_state.is_in_manipulation_manifold = false;
    action_node1.out_state.joint_values.clear();
    if(motion_trajectory.joint_trajectory.points.size() > 0)
    {
        for(int i = 0; i < motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions.size(); i++)
            action_node1.out_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions[i]);
    }

    action_node1.in_state.value = 0.0;
    action_node1.out_state.value = 0.0;

    action_node1.isBetweenFoliations = false;

    action_node1.motion_trajectory = motion_trajectory;

    action_node1.reward = reward;

    action_nodes.push_back(action_node1);

    manipulation_manifold_in_nodes[foliation_id][manipulation_manifold_id].push_back(current_action_node_id);
    intermediate_manifold_out_nodes[foliation_id][intermedaite_manifold_id].push_back(current_action_node_id);

    current_action_node_id++;

    // action node 2 is from intermediate manifold to manipulation manifold
    action_node2.action_node_id = current_action_node_id;
    action_node2.configuration_id = current_configuration_id;

    action_node2.in_state.foliation_id = foliation_id;
    action_node2.in_state.manifold_id = intermedaite_manifold_id;
    action_node2.in_state.is_in_manipulation_manifold = false;
    action_node2.in_state.joint_values.clear();
    if(motion_trajectory.joint_trajectory.points.size() > 0)
    {
        for(int i = 0; i < motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions.size(); i++)
            action_node2.in_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1].positions[i]);
    }

    action_node2.out_state.foliation_id = foliation_id;
    action_node2.out_state.manifold_id = manipulation_manifold_id;
    action_node2.out_state.is_in_manipulation_manifold = true;
    action_node2.out_state.joint_values.clear();
    if(motion_trajectory.joint_trajectory.points.size() > 0)
    {
        for(int i = 0; i < motion_trajectory.joint_trajectory.points[0].positions.size(); i++)
            action_node2.out_state.joint_values.push_back(motion_trajectory.joint_trajectory.points[0].positions[i]);
    }

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

    if(is_init_action)
    {
        init_action_node_ids.push_back(current_action_node_id);
    }

    current_action_node_id++;
    current_configuration_id++;
}

bool TaskPlanner::planActions(ActionSequence &action_sequence, const std::vector<float> &current_joint_value)
{
    if(init_action_node_ids.size() == 0)
    {
        std::cout << "no init action node found!" << std::endl;
        return false;
    }

    float init_value = -1000;
    int current_action_node_id = -1;
    for(int init_action_node_id: init_action_node_ids)
    {
        if(action_nodes[init_action_node_id].out_state.value > init_value)
        {
            init_value = action_nodes[init_action_node_id].out_state.value;
            current_action_node_id = init_action_node_id;
        }
    }

    if(init_value == -1000)
    {
        std::cout << "can't plan action sequence because of small init value!" << std::endl;
        return false;
    }

    // add the initial grasping motion into the action sequence.
    action_sequence.addActionTask(current_joint_value, 
                                 action_nodes[current_action_node_id].in_state.joint_values, 
                                 action_nodes[current_action_node_id].in_state.foliation_id,
                                 action_nodes[current_action_node_id].in_state.manifold_id,
                                 action_nodes[current_action_node_id].in_state.is_in_manipulation_manifold,
                                 action_nodes[current_action_node_id].motion_trajectory);

    // generate the action sequence
    int policy = action_nodes[current_action_node_id].policy;
    while(policy != -1)
    {
        int last_action_node_id = current_action_node_id;
        current_action_node_id = action_nodes[current_action_node_id].next_action_node_ids[policy];
        policy = action_nodes[current_action_node_id].policy;

        action_sequence.addActionTask(action_nodes[last_action_node_id].out_state.joint_values, 
                                     action_nodes[current_action_node_id].in_state.joint_values, 
                                     action_nodes[current_action_node_id].in_state.foliation_id,
                                     action_nodes[current_action_node_id].in_state.manifold_id,
                                     action_nodes[current_action_node_id].in_state.is_in_manipulation_manifold,
                                     action_nodes[current_action_node_id].motion_trajectory);
    }
    return true;
}

void TaskPlanner::updateTaskPlanner(const ActionSequence &action_sequence)
{

}

void TaskPlanner::constructMDPGraph()
{
    for(int i = 0; i < action_nodes.size(); i++)
    {
        action_nodes[i].next_action_node_ids.clear();
        
        if(action_nodes[i].isBetweenFoliations == false && action_nodes[i].in_state.is_in_manipulation_manifold == true)
        {// if the action node is from manipulation manifold to intermediate manifold
            for(long unsigned int possible_action_node_id: intermediate_manifold_in_nodes[action_nodes[i].out_state.foliation_id][action_nodes[i].out_state.manifold_id])
            {
                if(action_nodes[possible_action_node_id].configuration_id != action_nodes[i].configuration_id)
                {
                    action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
                    action_nodes[i].next_action_success_probabilities.push_back(0.5);
                }
            }
        } 
        else if(action_nodes[i].isBetweenFoliations == false && action_nodes[i].in_state.is_in_manipulation_manifold == false) 
        {// if the action node is to manipulation manifold
            for(long unsigned int possible_action_node_id: manipulation_manifold_in_nodes[action_nodes[i].out_state.foliation_id][action_nodes[i].out_state.manifold_id])
            {
                // TODO: based on the task, this statement may need to be changed.
                if(action_nodes[possible_action_node_id].isBetweenFoliations == true && action_nodes[i].isBetweenFoliations == false)
                {  // directly connect the intermedaite node to the related foliation node.
                    // only connect to the action node between foliation if it has the same joint values

                    if(action_nodes[possible_action_node_id].in_state.joint_values == action_nodes[i].out_state.joint_values)
                    {
                        action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
                        action_nodes[i].next_action_success_probabilities.push_back(0.5);
                    }
                }
                else if(action_nodes[possible_action_node_id].configuration_id != action_nodes[i].configuration_id)
                {
                    action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
                    action_nodes[i].next_action_success_probabilities.push_back(0.5);
                }
            }
        }
        else if(action_nodes[i].isBetweenFoliations == true) // if the action node is between foliations
        {
            for(long unsigned int possible_action_node_id: manipulation_manifold_in_nodes[action_nodes[i].out_state.foliation_id][action_nodes[i].out_state.manifold_id])
            {
                action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
                action_nodes[i].next_action_success_probabilities.push_back(0.5);
            }
        }
        // if current action node has no next action node, then its policy will be always -1.
        if(action_nodes[i].next_action_node_ids.size() > 0)
            action_nodes[i].policy = 0;
        else
            action_nodes[i].policy = -1;
    }
}

void TaskPlanner::policyIteration()
{
    int numOfUpdatedStates = 0;
    do
    {
        numOfUpdatedStates = 0;
        policyEvaluation();
        // policy improvement
        for(int i = 0; i < action_nodes.size(); i++)
        {
            if(action_nodes[i].reward > 0.0 || action_nodes[i].policy == -1)
                continue;
            int oldPolicy = action_nodes[i].policy;
            float maxQValue = -100000.0;
            for(int j = 0; j < action_nodes[i].next_action_node_ids.size(); j++)
            {
                float QValue = action_nodes[i].next_action_success_probabilities[j] * 
                               (action_nodes[action_nodes[i].next_action_node_ids[j]].reward + 
                                discount_factor * action_nodes[action_nodes[i].next_action_node_ids[j]].in_state.value);
                if(QValue > maxQValue)
                {
                    maxQValue = QValue;
                    action_nodes[i].policy = j;
                }
            }
            if(oldPolicy != action_nodes[i].policy)
                numOfUpdatedStates++;
        }
    }while(numOfUpdatedStates > 0);
    
}

void TaskPlanner::policyEvaluation()
{
    float error = 0.0;
    do
    {
        error = 0.0;
        for(int i = 0; i < action_nodes.size(); i++)
        {
            // it current action node has a positive reward or not a policy, the value of the state is the reward alone.
            if(action_nodes[i].reward > 0.0 || action_nodes[i].policy == -1)
            {
                action_nodes[i].out_state.value = action_nodes[i].reward;
            }
            else
            {
                int policyIndex = action_nodes[i].policy;
                action_nodes[i].out_state.value = action_nodes[i].next_action_success_probabilities[policyIndex] * (action_nodes[action_nodes[i].next_action_node_ids[policyIndex]].reward + discount_factor * action_nodes[action_nodes[i].next_action_node_ids[policyIndex]].in_state.value);
            }
        }

        for(int i = 0; i < action_nodes.size(); i++)
        {
            error = std::max(error, fabs(action_nodes[i].out_state.value - action_nodes[i].in_state.value));
            action_nodes[i].in_state.value = action_nodes[i].out_state.value;
        }
    }while(error > max_error * (1 - discount_factor) / discount_factor);
}
    
int TaskPlanner::getFoliationSize()
{
    return number_of_foliations;
}