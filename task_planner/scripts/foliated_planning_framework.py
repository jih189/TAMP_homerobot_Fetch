

class FoliatedPlanningFramework():
    '''
    This class implements the foliated planning framework. In this class, the framework will call both task planner
    and motion planner to solve the problem with foliated structure.
    '''
    def __init__(self, task_planner, motion_planner):
        self.task_planner = task_planner
        self.motion_planner = motion_planner
        self.max_attempt_time = 10
        self.has_visualizer = False

    def setFoliatedProblem(self, foliated_problem):
        '''
        This function sets the foliated problem to the planning framework.
        '''
        self.foliated_problem = foliated_problem

    def setMotionPlanner(self, motion_planner):
        '''
        This function sets the motion planner to the planning framework.
        '''
        self.motion_planner = motion_planner
        self.prepare_planner()

    def setTaskPlanner(self, task_planner):
        '''
        This function sets the task planner to the planning framework.
        '''
        self.task_planner = task_planner

    def setVisualizer(self, visualizer):
        '''
        This function sets the visualizer to the planning framework.
        '''
        self.visualizer = visualizer
        self.has_visualizer = True

    def setMaxAttemptTime(self, max_attempt_time=10):
        '''
        This function sets the maximum attempt time for the planning framework.
        '''
        self.max_attempt_time = max_attempt_time

    def setStartAndGoal(self, start_foliation_index, start_co_parameter_index, start_configuration, goal_foliation_index, goal_co_parameter_index, goal_configuration):
        '''
        This function sets the start and goal configuration to the planning framework.
        '''
        self.start_foliation_index = start_foliation_index
        self.start_co_parameter_index = start_co_parameter_index
        self.goal_foliation_index = goal_foliation_index
        self.goal_co_parameter_index = goal_co_parameter_index
        self.start_configuration = start_configuration
        self.goal_configuration = goal_configuration
    
    def solve(self):
        '''
        This function solves the problem with foliated structure.
        If the solution is found, the framework will return a list of motion plan for each task in sequence.
        That is, the result is a list of motion plan, and the motion planner need to consider it as a list for
        visualization later.
        '''
        # reset the task planner
        self.task_planner.reset_task_planner()

        # load the foliated problem
        self.task_planner.load_foliated_problem(self.foliated_problem)

        # set the start and goal
        self.task_planner.set_start_and_goal(
            (self.start_foliation_index, self.start_co_parameter_index),
            self.start_configuration,
            (self.goal_foliation_index, self.goal_co_parameter_index),
            self.goal_configuration
        )

        for attempt_time in range(self.max_attempt_time):

            # generate the task sequence
            task_sequence = self.task_planner.generate_task_sequence()

            # print detail of generetated task_sequence
            # for t, task in enumerate(task_sequence):
            #     print "task ", t, "-----------------------"
            #     print "foliation name: ", task.manifold_detail.foliation.foliation_name
                # print "start configuration"
                # print(task.start_configuration)
                # print "goal configuration"
                # print(task.goal_configuration)

            if len(task_sequence) == 0:
                return False, None

            list_of_motion_plan = []
            found_solution = True

            # solve the problem
            for task in task_sequence:

                # try to visualize the task hints which is a list of distributions
                print "distribution list"
                for distribution in task.distributions:
                    print distribution.mean
                self.visualizer.visualize_distribution(task.distributions)

                # if task.has_solution:
                #     list_of_motion_plan.append(task.solution_trajectory)
                # else:
                # plan the motion
                success_flag, motion_plan_result, experience = self.motion_planner._plan(
                    task.start_configuration, 
                    task.goal_configuration, 
                    task.manifold_detail.foliation.constraint_parameters, 
                    task.manifold_detail.foliation.co_parameters[task.manifold_detail.co_parameter_index],
                    task.distributions
                )

                self.task_planner.update(task.task_graph_info, experience)

                if success_flag:
                    list_of_motion_plan.append(motion_plan_result)
                    # add the intersection action to the list of motion plan
                    list_of_motion_plan.append(task.next_motion.get_task_motion())
                else:
                    found_solution = False
                    break

            if not found_solution:
                continue
            else:
                return True, list_of_motion_plan

        return False, None

    def visualizeSolutionTrajectory(self, list_of_motion_plan):
        '''
        This function visualizes the solution path.
        '''
        if self.has_visualizer:
            self.visualizer._visualize_plan(list_of_motion_plan)
        else:
            raise Exception("No visualizer is set to the planning framework.")

    def shutdown(self):
        '''
        This function shuts down the planning framework.
        '''
        # self.task_planner.shutdown_task_planner()
        self.motion_planner.shutdown_planner()

   