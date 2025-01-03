import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        self.max_time_horizon = len(my_map) * len(my_map[0])

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = [
            # {
            #     # Task 1.2: Vertex constraint for Agent 0
            #     'agent': 0, 'loc': [(1, 5)], 'timestep': 4
            # },
            # {
            #     # Task 1.3: Edge constraint for Agent 1
            #     'agent': 1, 'loc': [(1, 2), (1, 3)], 'timestep': 1
            # },
            # {  # Task 1.4: Handling Goal Constraints
            #     'agent': 0, 'loc': [(1, 5)], 'timestep': 10},
            #
            # # Task 1.5 Designed Constraints
            # {'agent': 1, 'loc': [(1, 4)], 'timestep': 3},
            # {'agent': 2, 'loc': [(2, 4), (2, 5)], 'timestep': 4},
            # {'agent': 1, 'loc': [(2, 2)], 'timestep': 2}
        ]
        # Calculate upper bounds on path lengths for each agent
        upper_bounds = [self.max_time_horizon for _ in range(self.num_of_agents)]

        for i in range(self.num_of_agents):  # Find path for each agent
            max_steps = min(upper_bounds[i], self.max_time_horizon)
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)

            if path is None:
                print(f"No solutions for Agent {i} within {max_steps} steps.")
                return None  # Gracefully return without raising an exception

            result.append(path)
            upper_bounds[i] = len(path)

            # Add vertex and edge constraints for future agents
            # for t, loc in enumerate(path):
            #     for j in range(i + 1, self.num_of_agents):
            #         constraints.append({'agent': j, 'loc': [loc], 'timestep': t})
            #
            #     # Add edge constraints to prevent swaps
            # for t in range(1, len(path)):
            #     prev_loc = path[t - 1]
            #     curr_loc = path[t]
            #     for j in range(i + 1, self.num_of_agents):
            #         constraints.append({'agent': j, 'loc': [prev_loc, curr_loc], 'timestep': t})
            #         constraints.append({'agent': j, 'loc': [curr_loc, prev_loc], 'timestep':t})
            goal_location = self.goals[i]
            goal_time = len(path) - 1

            for j in range(i + 1, self.num_of_agents):
                for t in range(goal_time, self.max_time_horizon):
                    constraints.append({
                        'agent': j,
                        'loc': [goal_location],
                        'timestep': t
                    })

                # Add usual vertex and edge constraints
            for t, loc in enumerate(path):
                for j in range(i + 1, self.num_of_agents):
                    # Add vertex constraint
                    constraints.append({'agent': j, 'loc': [loc], 'timestep': t})

                    # Add edge constraints
                    if t > 0:
                        prev_loc = path[t - 1]
                        constraints.append({'agent': j, 'loc': [prev_loc, loc], 'timestep': t})
                        constraints.append({'agent': j, 'loc': [loc, prev_loc], 'timestep': t})

                # Update the upper bounds for remaining agents
            upper_bounds[i] = len(path)

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
