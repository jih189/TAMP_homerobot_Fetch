#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graphviz.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>

struct MotionTask
    {
        std::vector<float> start_joint_values;
        std::vector<float> target_joint_values;
        long unsigned int foliation_id;
        long unsigned int manifold_id;
        bool is_in_manipulation_manifold;
        moveit_msgs::RobotTrajectory solution_trajectory;
    };

/**
Class action sequence is generate by the task planner initially, then it will be passed to motion planner
to generate the motion trajectory by filling the missing information.
*/
class ActionSequence
{
    public:
    ActionSequence()
    {
        motion_tasks.clear();
        cartesian_motions.clear();
        previous_node_ids.clear();
        next_node_ids.clear();
        has_solutions.clear();
        in_hand_poses.clear();
    }

    void addActionTask(const std::vector<float> &start_joint_values,
                       const std::vector<float> &target_joint_values,
                       long unsigned int foliation_id,
                       long unsigned int manifold_id,
                       bool is_in_manipulation_manifold,
                       const moveit_msgs::RobotTrajectory &cartesian_motion,
                       bool isToNextFoliation,
                       int previous_node_id, 
                       int next_node_id, 
                       bool has_solution,
                       const geometry_msgs::Pose &in_hand_pose)
    {
        MotionTask motion_task;
        for(int i = 0; i < start_joint_values.size(); i++)
            motion_task.start_joint_values.push_back(start_joint_values[i]);
        for(int i = 0; i < target_joint_values.size(); i++)
            motion_task.target_joint_values.push_back(target_joint_values[i]);

        motion_task.foliation_id = foliation_id;
        motion_task.manifold_id = manifold_id;
        motion_task.is_in_manipulation_manifold = is_in_manipulation_manifold;

        motion_tasks.push_back(motion_task);

        cartesian_motions.push_back(cartesian_motion);

        isToNextFoliations.push_back(isToNextFoliation);

        previous_node_ids.push_back(previous_node_id);
        next_node_ids.push_back(next_node_id);
        has_solutions.push_back(has_solution);

        in_hand_poses.push_back(in_hand_pose);
    }

    long unsigned int getActionSize() const
    {
        return motion_tasks.size();
    }

    MotionTask getActionTaskAt(long unsigned int task_index) const
    {
        return motion_tasks[task_index];
    }

    geometry_msgs::Pose getInHandPoseAt(long unsigned int task_index) const
    {
        return in_hand_poses[task_index];
    }

    void setSolutionForActionTaskAt(int task_index, const moveit_msgs::RobotTrajectory &solution_motion, const geometry_msgs::Pose &in_hand_pose){
        motion_tasks[task_index].solution_trajectory = solution_motion;
        in_hand_poses[task_index] = in_hand_pose;
        setHasSolutionAt(task_index, true);
    }

    moveit_msgs::RobotTrajectory getCartesianMotionAt(long unsigned int task_index){
        return cartesian_motions[task_index];
    }

    bool isToNextFoliationAt(long unsigned int task_index) const
    {
        return isToNextFoliations[task_index];
    }

    int getPreviousIdAt(long unsigned int task_index) const
    {
        return previous_node_ids[task_index];
    }

    int getNextIdAt(long unsigned int task_index) const
    {
        return next_node_ids[task_index];
    }

    bool hasSolutionAt(long unsigned int task_index) const
    {
        return has_solutions[task_index];
    }

    void setHasSolutionAt(long unsigned int task_index, bool has_solution)
    {
        has_solutions[task_index] = has_solution;
    }

    private:

    std::vector<MotionTask> motion_tasks;
    std::vector<moveit_msgs::RobotTrajectory> cartesian_motions;
    std::vector<bool> isToNextFoliations;
    std::vector<int> previous_node_ids;
    std::vector<int> next_node_ids;
    std::vector<bool> has_solutions;
    std::vector<geometry_msgs::Pose> in_hand_poses;
};

class TaskPlanner
{
    struct ActionState
    {
        long unsigned int foliation_id;
        long unsigned int manifold_id;
        bool is_in_manipulation_manifold;
        std::vector<float> joint_values;
        float value;
    };

    struct ActionNode
    {
        long unsigned int action_node_id;
        long unsigned int configuration_id;
        ActionState in_state;
        ActionState out_state;
        float reward;
        bool isBetweenFoliations;
        moveit_msgs::RobotTrajectory motion_trajectory; // the cartesian motion trajectory between two states
        std::vector<long unsigned int> next_action_node_ids;
        std::vector<float> next_action_success_probabilities;
        long unsigned int policy;
        std::vector<moveit_msgs::RobotTrajectory> solution_trajectories;
        std::vector<geometry_msgs::Pose> in_hand_poses;
        std::vector<bool> has_solution;
    };
    
    public:
    TaskPlanner(std::vector<long unsigned int> number_of_intermediate_manifolds_, std::vector<long unsigned int> number_of_manipulation_manifolds_);

    // if the action is between two foliation, the motion trajectory should be start from last foliation and end at the next foliation.    
    void addActionBetweenFoliations(moveit_msgs::RobotTrajectory motion_trajectory, long unsigned int last_foliation_id, long unsigned int last_manipulation_id, long unsigned int next_foliation_id, long unsigned int next_manipulation_id, float reward);

    // if the action is between two manifolds, the motion trajectory should be start from intermediate manifold and end at the next manipulation manifold.
    void addActionBetweenManifolds(moveit_msgs::RobotTrajectory motion_trajectory, long unsigned int intermedaite_manifold_id, long unsigned int manipulation_manifold_id, long unsigned int foliation_id, bool is_init_action, float reward);
    
    void policyIteration();

    void policyEvaluation();

    bool planActions(ActionSequence &action_sequence, const std::vector<float> &current_joint_value);

    // based on the solution in the action sequence, update the task planner
    void updateTaskPlanner(const ActionSequence &action_sequence);

    void constructMDPGraph();

    int getFoliationSize();

    private:
    long unsigned int current_configuration_id;
    long unsigned int current_action_node_id;
    std::vector<long unsigned int> number_of_intermediate_manifolds;
    std::vector<long unsigned int> number_of_manipulation_manifolds;
    long unsigned int number_of_foliations;

    std::vector<std::vector<std::vector<long unsigned int>>> manipulation_manifold_out_nodes;
    std::vector<std::vector<std::vector<long unsigned int>>> manipulation_manifold_in_nodes;

    std::vector<std::vector<std::vector<long unsigned int>>> intermediate_manifold_out_nodes;
    std::vector<std::vector<std::vector<long unsigned int>>> intermediate_manifold_in_nodes;

    std::vector<long unsigned int> init_action_node_ids;

    std::vector<ActionNode> action_nodes;
    float discount_factor;
    float max_error;
};