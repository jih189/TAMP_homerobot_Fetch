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
        // action_node2.motion_trajectory.joint_trajectory.points[i].time_from_start = motion_trajectory.joint_trajectory.points[motion_trajectory.joint_trajectory.points.size() - 1 - i].time_from_start;
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

void TaskPlanner::addGoalInManifolds(long unsigned int goal_manipulation_id, long unsigned int goal_foliation_id, float reward)
{
    TaskPlanner::ActionNode goal_action_node;

    // action node 1 is from manipulation manifold to intermediate manifold
    goal_action_node.action_node_id = current_action_node_id;
    goal_action_node.configuration_id = current_configuration_id;

    goal_action_node.in_state.foliation_id = goal_foliation_id;
    goal_action_node.in_state.manifold_id = goal_manipulation_id;
    goal_action_node.in_state.is_in_manipulation_manifold = true;
    goal_action_node.in_state.joint_values.clear();

    goal_action_node.out_state.foliation_id = goal_foliation_id;
    goal_action_node.out_state.manifold_id = goal_manipulation_id;
    goal_action_node.out_state.is_in_manipulation_manifold = true;
    goal_action_node.out_state.joint_values.clear();

    goal_action_node.in_state.value = 0.0;
    goal_action_node.out_state.value = 0.0;

    goal_action_node.isBetweenFoliations = false;

    goal_action_node.reward = reward;

    action_nodes.push_back(goal_action_node);

    manipulation_manifold_in_nodes[goal_foliation_id][goal_manipulation_id].push_back(current_action_node_id);

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
                                 action_nodes[current_action_node_id].motion_trajectory,
                                 action_nodes[current_action_node_id].in_state.foliation_id != action_nodes[current_action_node_id].out_state.foliation_id,
                                 -1,
                                 current_action_node_id,
                                 false,
                                 geometry_msgs::Pose());
    
    int lenth_of_action_sequence = 0;

    // generate the action sequence
    int policy = action_nodes[current_action_node_id].policy;
    while(policy != -1)
    {
        int last_action_node_id = current_action_node_id;
        current_action_node_id = action_nodes[current_action_node_id].next_action_node_ids[policy];

        // if the current action node is target action node
        if(action_nodes[current_action_node_id].isBetweenFoliations == false && 
           action_nodes[current_action_node_id].in_state.is_in_manipulation_manifold == true &&
           action_nodes[current_action_node_id].out_state.is_in_manipulation_manifold == true)
        {
            action_sequence.addGoalActionTask(action_nodes[last_action_node_id].out_state.joint_values,
                                        action_nodes[current_action_node_id].in_state.foliation_id,
                                        action_nodes[current_action_node_id].in_state.manifold_id,
                                        action_nodes[current_action_node_id].in_state.is_in_manipulation_manifold,
                                        action_nodes[current_action_node_id].in_state.foliation_id != action_nodes[current_action_node_id].out_state.foliation_id,
                                        last_action_node_id,
                                        current_action_node_id, 
                                        false,
                                        geometry_msgs::Pose());
        }
        else{
            action_sequence.addActionTask(action_nodes[last_action_node_id].out_state.joint_values, 
                                        action_nodes[current_action_node_id].in_state.joint_values, 
                                        action_nodes[current_action_node_id].in_state.foliation_id,
                                        action_nodes[current_action_node_id].in_state.manifold_id,
                                        action_nodes[current_action_node_id].in_state.is_in_manipulation_manifold,
                                        action_nodes[current_action_node_id].motion_trajectory,
                                        action_nodes[current_action_node_id].in_state.foliation_id != action_nodes[current_action_node_id].out_state.foliation_id,
                                        last_action_node_id,
                                        current_action_node_id, 
                                        false,
                                        geometry_msgs::Pose());

            // if solution exists in this step, then reuse this solution.
            if(action_nodes[last_action_node_id].has_solution[policy])
            {
                int current_index = action_sequence.getActionSize() - 1;
                action_sequence.setSolutionForActionTaskAt(current_index, action_nodes[last_action_node_id].solution_trajectories[policy], action_nodes[last_action_node_id].in_hand_poses[policy]);
            }            
        }

        policy = action_nodes[current_action_node_id].policy;
        
        lenth_of_action_sequence++;
        if(lenth_of_action_sequence > 100)
        {
            std::cout << "the length of action sequence is too long!" << std::endl;
            return false;
        }
    }
    return true;
}

void TaskPlanner::updateTaskPlanner(const ActionSequence &action_sequence)
{
    for(int i = 0; i < action_sequence.getActionSize(); i++)
    {
        // skip the initial grasping motion.
        if(action_sequence.getPreviousIdAt(i) == -1)
            continue;
        
        // get the two end nodes of the current edge in task graph.
        int previous_node_id = action_sequence.getPreviousIdAt(i);
        int next_node_id = action_sequence.getNextIdAt(i);
        if(not action_sequence.hasSolutionAt(i)) // this is no solution in this step.
        {
            // update the success rate of current edge in task graph.
            std::vector<long unsigned int>::iterator itr = std::find(action_nodes[previous_node_id].next_action_node_ids.begin(), action_nodes[previous_node_id].next_action_node_ids.end(), next_node_id);
            int policyIndexForNextNode = std::distance(action_nodes[previous_node_id].next_action_node_ids.begin(), itr);
            action_nodes[previous_node_id].next_action_success_probabilities[policyIndexForNextNode] *= 0.85;

            // update the success rate in opposte edge too
            itr = std::find(action_nodes[next_node_id].next_action_node_ids.begin(), action_nodes[next_node_id].next_action_node_ids.end(), previous_node_id);
            // if the opposite edge is not found, the this edge is between foliation.
            if(itr == action_nodes[next_node_id].next_action_node_ids.end())
            {
                break;
            }
            policyIndexForNextNode = std::distance(action_nodes[next_node_id].next_action_node_ids.begin(), itr);
            action_nodes[next_node_id].next_action_success_probabilities[policyIndexForNextNode] *= 0.85;

            // need to update the success rate of similar edges in task graph.
            //TODO
            long unsigned int failed_pre_node_manifold_id = action_nodes[previous_node_id].in_state.manifold_id;
            long unsigned int failed_pre_node_foliation_id = action_nodes[previous_node_id].in_state.foliation_id;
            bool failed_pre_node_is_in_manipulation_manifold = action_nodes[previous_node_id].in_state.is_in_manipulation_manifold;

            long unsigned int failed_next_node_manifold_id = action_nodes[next_node_id].out_state.manifold_id;
            long unsigned int failed_next_node_foliation_id = action_nodes[next_node_id].out_state.foliation_id;
            bool failed_next_node_is_in_manipulation_manifold = action_nodes[next_node_id].out_state.is_in_manipulation_manifold;

            // long unsigned int failed_current_node_manifold_id = action_nodes[next_node_id].in_state.manifold_id;
            if(
                action_nodes[previous_node_id].out_state.manifold_id != action_nodes[next_node_id].in_state.manifold_id ||
                action_nodes[previous_node_id].out_state.foliation_id != action_nodes[next_node_id].in_state.foliation_id ||
                action_nodes[previous_node_id].out_state.is_in_manipulation_manifold != action_nodes[next_node_id].in_state.is_in_manipulation_manifold
            )
            {
                std::cerr << "-----------------------------------------------------" << std::endl;
                std::cerr << "something wrong in updateTaskPlanner" << std::endl;
                std::cerr << "-----------------------------------------------------" << std::endl;
            }

            long unsigned int failed_current_node_manifold_id = action_nodes[previous_node_id].out_state.manifold_id;
            long unsigned int failed_current_node_foliation_id = action_nodes[previous_node_id].out_state.foliation_id;
            bool failed_current_node_is_in_manipulation_manifold = action_nodes[previous_node_id].out_state.is_in_manipulation_manifold;
            

            // loop through all action nodes
            for(int j = 0; j < action_nodes.size(); j++)
            {
                if(action_nodes[j].in_state.manifold_id == failed_pre_node_manifold_id &&
                   action_nodes[j].in_state.foliation_id == failed_pre_node_foliation_id &&
                   action_nodes[j].in_state.is_in_manipulation_manifold == failed_pre_node_is_in_manipulation_manifold)
                {
                    // find all next nodes of this node
                    for(int k = 0; k < action_nodes[j].next_action_node_ids.size(); k++)
                    {
                        if(action_nodes[j].next_action_success_probabilities[k] == 1.0) // if there is a solution, then skip.
                            continue;
                        if(
                            action_nodes[action_nodes[j].next_action_node_ids[k]].out_state.manifold_id == failed_next_node_manifold_id &&
                            action_nodes[action_nodes[j].next_action_node_ids[k]].out_state.foliation_id == failed_next_node_foliation_id &&
                            action_nodes[action_nodes[j].next_action_node_ids[k]].out_state.is_in_manipulation_manifold == failed_next_node_is_in_manipulation_manifold
                        )
                        {
                            if(action_nodes[j].out_state.manifold_id == failed_current_node_foliation_id &&
                               action_nodes[j].out_state.foliation_id == failed_current_node_foliation_id &&
                               action_nodes[j].out_state.is_in_manipulation_manifold == failed_current_node_is_in_manipulation_manifold)
                            {
                                // the edge in the same manifold and share two edge end
                                action_nodes[j].next_action_success_probabilities[k] *= 0.8;
                            }
                            else{
                                // the edge in different manifold and share two edge end
                                action_nodes[j].next_action_success_probabilities[k] *= 0.9;
                            }
                        }
                    }
                }

                // in opposite direction
                if(action_nodes[j].in_state.manifold_id == failed_next_node_manifold_id &&
                   action_nodes[j].in_state.foliation_id == failed_next_node_foliation_id &&
                   action_nodes[j].in_state.is_in_manipulation_manifold == failed_next_node_is_in_manipulation_manifold)
                {
                    // find all next nodes of this node
                    for(int k = 0; k < action_nodes[j].next_action_node_ids.size(); k++)
                    {
                        if(action_nodes[j].next_action_success_probabilities[k] == 1.0) // if there is a solution, then skip.
                            continue;
                        if(
                            action_nodes[action_nodes[j].next_action_node_ids[k]].out_state.manifold_id == failed_pre_node_manifold_id &&
                            action_nodes[action_nodes[j].next_action_node_ids[k]].out_state.foliation_id == failed_pre_node_foliation_id &&
                            action_nodes[action_nodes[j].next_action_node_ids[k]].out_state.is_in_manipulation_manifold == failed_pre_node_is_in_manipulation_manifold
                        )
                        {
                            if(action_nodes[j].out_state.manifold_id == failed_current_node_foliation_id &&
                               action_nodes[j].out_state.foliation_id == failed_current_node_foliation_id &&
                               action_nodes[j].out_state.is_in_manipulation_manifold == failed_current_node_is_in_manipulation_manifold)
                            {
                                // the edge in the same manifold and share two edge end
                                action_nodes[j].next_action_success_probabilities[k] *= 0.8;
                            }
                            else{
                                // the edge in different manifold and share two edge end
                                action_nodes[j].next_action_success_probabilities[k] *= 0.9;
                            }
                        }
                    }
                }
            }

            // only update the first failure task.
            break;
        }
        else{
            // set the current edge in task graph has solution.
            std::vector<long unsigned int>::iterator itr = std::find(action_nodes[previous_node_id].next_action_node_ids.begin(), action_nodes[previous_node_id].next_action_node_ids.end(), next_node_id);
            int policyIndexForNextNode = std::distance(action_nodes[previous_node_id].next_action_node_ids.begin(), itr);

            // if the next action node is a goal action node, then do not set solution here.
            if(action_nodes[next_node_id].isBetweenFoliations == false && action_nodes[next_node_id].in_state.is_in_manipulation_manifold == true && action_nodes[next_node_id].out_state.is_in_manipulation_manifold == true)
            {
                action_nodes[previous_node_id].next_action_success_probabilities[policyIndexForNextNode] = 1.0;
                action_nodes[previous_node_id].in_hand_poses[policyIndexForNextNode] = action_sequence.getInHandPoseAt(i);
                continue;
            }
            
            // if current edge in task graph has solution then skip
            if(action_nodes[previous_node_id].has_solution[policyIndexForNextNode])
                continue;

            action_nodes[previous_node_id].has_solution[policyIndexForNextNode] = true;
            action_nodes[previous_node_id].next_action_success_probabilities[policyIndexForNextNode] = 1.0;
            action_nodes[previous_node_id].solution_trajectories[policyIndexForNextNode] = action_sequence.getActionTaskAt(i).solution_trajectory;
            action_nodes[previous_node_id].in_hand_poses[policyIndexForNextNode] = action_sequence.getInHandPoseAt(i);
            
            // update the success rate in opposte edge too
            itr = std::find(action_nodes[next_node_id].next_action_node_ids.begin(), action_nodes[next_node_id].next_action_node_ids.end(), previous_node_id);
            // if the opposite edge is not found, the this edge is between foliation.
            if(itr == action_nodes[next_node_id].next_action_node_ids.end())
            {
                continue;
            }
            policyIndexForNextNode = std::distance(action_nodes[next_node_id].next_action_node_ids.begin(), itr);
            
            action_nodes[next_node_id].has_solution[policyIndexForNextNode] = true;
            action_nodes[next_node_id].next_action_success_probabilities[policyIndexForNextNode] = 1.0;
            action_nodes[next_node_id].solution_trajectories[policyIndexForNextNode] = action_sequence.getActionTaskAt(i).solution_trajectory;
            action_nodes[next_node_id].in_hand_poses[policyIndexForNextNode] = action_sequence.getInHandPoseAt(i);
            
            // need to reverse the trajectory
            for(int p = 0; p < action_sequence.getActionTaskAt(i).solution_trajectory.joint_trajectory.points.size(); p++)
            {
                for(int j = 0; j < action_sequence.getActionTaskAt(i).solution_trajectory.joint_trajectory.points[p].positions.size(); j++)
                {
                    action_nodes[next_node_id].solution_trajectories[policyIndexForNextNode].joint_trajectory.points[p].positions[j] = action_sequence.getActionTaskAt(i).solution_trajectory.joint_trajectory.points[action_sequence.getActionTaskAt(i).solution_trajectory.joint_trajectory.points.size() - 1 - p].positions[j];
                }
                // action_nodes[next_node_id].solution_trajectories[policyIndexForNextNode].joint_trajectory.points[p].time_from_start = action_sequence.getActionTaskAt(i).solution_trajectory.joint_trajectory.points[action_sequence.getActionTaskAt(i).solution_trajectory.joint_trajectory.points.size() - 1 - p].time_from_start;
            }            
        }
    }
}

void TaskPlanner::constructMDPGraph()
{
    for(int i = 0; i < action_nodes.size(); i++)
    {
        action_nodes[i].next_action_node_ids.clear();

        if(action_nodes[i].isBetweenFoliations == false && action_nodes[i].in_state.is_in_manipulation_manifold == true && action_nodes[i].out_state.is_in_manipulation_manifold == false)
        {// if the action node is from manipulation manifold to intermediate manifold
            for(long unsigned int possible_action_node_id: intermediate_manifold_in_nodes[action_nodes[i].out_state.foliation_id][action_nodes[i].out_state.manifold_id])
            {
                if(action_nodes[possible_action_node_id].configuration_id != action_nodes[i].configuration_id)
                {
                    action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
                    action_nodes[i].next_action_success_probabilities.push_back(0.5);
                    action_nodes[i].solution_trajectories.push_back(moveit_msgs::RobotTrajectory());
                    action_nodes[i].in_hand_poses.push_back(geometry_msgs::Pose());
                    action_nodes[i].has_solution.push_back(false);
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
                        action_nodes[i].solution_trajectories.push_back(moveit_msgs::RobotTrajectory());
                        action_nodes[i].in_hand_poses.push_back(geometry_msgs::Pose());
                        action_nodes[i].has_solution.push_back(false);
                    }
                }
                else if(action_nodes[possible_action_node_id].configuration_id != action_nodes[i].configuration_id)
                {
                    action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
                    action_nodes[i].next_action_success_probabilities.push_back(0.5);
                    action_nodes[i].solution_trajectories.push_back(moveit_msgs::RobotTrajectory());
                    action_nodes[i].in_hand_poses.push_back(geometry_msgs::Pose());
                    action_nodes[i].has_solution.push_back(false);
                }
            }
        }
        else if(action_nodes[i].isBetweenFoliations == true) // if the action node is between foliations
        {
            for(long unsigned int possible_action_node_id: manipulation_manifold_in_nodes[action_nodes[i].out_state.foliation_id][action_nodes[i].out_state.manifold_id])
            {
                action_nodes[i].next_action_node_ids.push_back(possible_action_node_id);
                action_nodes[i].next_action_success_probabilities.push_back(0.5);
                action_nodes[i].solution_trajectories.push_back(moveit_msgs::RobotTrajectory());
                action_nodes[i].in_hand_poses.push_back(geometry_msgs::Pose());
                action_nodes[i].has_solution.push_back(false);
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
        // std::cout << "numOfUpdatedStates: " << numOfUpdatedStates << std::endl;
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