

class FoliatedPlanningFramework():
    '''
    This class implements the foliated planning framework. In this class, the framework will call both task planner
    and motion planner to solve the problem with foliated structure.
    '''
    def __init__(self):
        pass

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

    def setTaskPlanner(self, task_planner):
        '''
        This function sets the task planner to the planning framework.
        '''
        self.task_planner = task_planner

    def setMaxAttemptTime(self, max_attempt_time):
        '''
        This function sets the maximum attempt time for the planning framework.
        '''
        self.max_attempt_time = max_attempt_time

    def setStartAndGoal(self, start_foliation_id, start_co_parameter_index, start_configuration, goal_foliation_id, goal_co_parameter_index, goal_configuration):
        '''
        This function sets the start and goal configuration to the planning framework.
        '''
        self.start_foliation_id = start_foliation_id
        self.start_co_parameter_index = start_co_parameter_index
        self.goal_foliation_id = goal_foliation_id
        self.goal_co_parameter_index = goal_co_parameter_index
        self.start_configuration = start_configuration
        self.goal_configuration = goal_configuration
    
    def solve(self):
        '''
        This function solves the problem with foliated structure.
        '''
        # reset the task planner
        self.task_planner.reset_task_planner()

        # load the foliated problem
        self.task_planner.load_foliated_problem(self.foliated_problem)

        # set the start and goal
        self.task_planner.set_start_and_goal(
            (self.start_foliation_id, self.start_co_parameter_index),
            self.start_configuration,
            (self.goal_foliation_id, self.goal_co_parameter_index),
            self.goal_configuration
        )

        for attempt_time in range(self.max_attempt_time):

            # generate the task sequence
            task_sequence = self.task_planner.generate_task_sequence()

            if len(task_sequence) == 0:
                return False, None

            solution_path = []
            found_solution = True

            # solve the problem
            for task in task_sequence:

                if task.has_solution:
                    solution_path.append(task.solution_trajectory)
                else:
                    # plan the motion
                    planning_feedback, motion_plan_result = self.motion_planner.plan(
                        task.start_configuration, 
                        task.goal_configuration, 
                        task.manifold_detail.foliation.constraint_parameters, 
                        task.next_motion
                    )

                    task_planner.update(planning_feedback)

                    if not motion_plan_result:
                        found_solution = False
                        break
                    else:
                        solution_path.append(motion_plan_result)

                if not found_solution:
                    continue
                else:
                    return True, solution_path

        return False, None


   