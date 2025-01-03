import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost, lpa_star


def detect_collision(path1, path2):
    ##############################
    # Task 3.1: Return the first collision that occurs between two robot paths (or None if there is no collision)
    #           There are two types of collisions: vertex collision and edge collision.
    #           A vertex collision occurs if both robots occupy the same location at the same timestep
    #           An edge collision occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.
    max_t = max(len(path1), len(path2))

    for t in range(max_t):
        loc1 = get_location(path1, t)
        loc2 = get_location(path2, t)

        # Check for vertex collision
        if loc1 == loc2:
            print(f"Vertex collision detected at timestep {t} at location {loc1}")
            return {'loc': [loc1], 'timestep': t}

        # Check for edge collision
        if t > 0:
            prev_loc1 = get_location(path1, t - 1)
            prev_loc2 = get_location(path2, t - 1)
            if prev_loc1 == loc2 and prev_loc2 == loc1:
                print(f"Edge collision detected at timestep {t} between locations {prev_loc1} and {loc1}")
                return {'loc': [prev_loc1, loc1], 'timestep': t}

    return None  # No collision found

def detect_collisions(paths):
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    collisions = []
    num_agents = len(paths)

    for i in range(num_agents):
        for j in range(i + 1, num_agents):
            collision = detect_collision(paths[i], paths[j])
            if collision:
                collision['a1'] = i
                collision['a2'] = j
                print(f"Collision detected between agents {i} and {j}: {collision}")
                collisions.append(collision)

    return collisions

def standard_splitting(collision):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge collision: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep
    a1 = collision['a1']
    a2 = collision['a2']
    loc = collision['loc']
    timestep = collision['timestep']

    if len(loc) == 1:
        # Vertex collision
        constraints = [
            {'agent': a1, 'loc': [loc[0]], 'timestep': timestep},
            {'agent': a2, 'loc': [loc[0]], 'timestep': timestep}
        ]
    else:
        # Edge collision
        constraints = [
            {'agent': a1, 'loc': [loc[0], loc[1]], 'timestep': timestep},
            {'agent': a2, 'loc': [loc[1], loc[0]], 'timestep': timestep}
        ]

    return constraints


def disjoint_splitting(collision):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given collision
    #           Vertex collision: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge collision: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly
    loc = collision['loc']
    timestep = collision['timestep']
    agent1 = collision['a1']
    agent2 = collision['a2']

    # Decide randomly which agent to create a positive constraint for
    positive_agent = random.randint(0, 1)  # 0 for agent1, 1 for agent2

    # Prepare the constraints
    constraints = []

    # Vertex collision (single location)
    if len(loc) == 1:
        # Create constraints for a vertex collision
        if positive_agent == 0:
            constraints.append({'agent': agent1, 'loc': [loc[0]], 'timestep': timestep, 'positive': True})
            constraints.append({'agent': agent2, 'loc': [loc[0]], 'timestep': timestep, 'positive': False})
        else:
            constraints.append({'agent': agent1, 'loc': [loc[0]], 'timestep': timestep, 'positive': False})
            constraints.append({'agent': agent2, 'loc': [loc[0]], 'timestep': timestep, 'positive': True})

    # Edge collision (two locations)
    elif len(loc) == 2:
        # Create constraints for an edge collision
        if positive_agent == 0:
            constraints.append({'agent': agent1, 'loc': [loc[0], loc[1]], 'timestep': timestep, 'positive': True})
            constraints.append({'agent': agent2, 'loc': [loc[1], loc[0]], 'timestep': timestep, 'positive': False})
        else:
            constraints.append({'agent': agent1, 'loc': [loc[0], loc[1]], 'timestep': timestep, 'positive': False})
            constraints.append({'agent': agent2, 'loc': [loc[1], loc[0]], 'timestep': timestep, 'positive': True})

    return constraints

# the given helper function
def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst

#base CBS
'''
class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        self.start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)
        #
        while len(self.open_list) > 0:
            # Pop the node with the lowest cost
            node = self.pop_node()
            print(f"Expanding node with cost {node['cost']} and {len(node['collisions'])} collisions")

            # If no collisions, return solution
            if not node['collisions']:
                self.print_results(node)
                return node['paths']

            # Get the first collision and create constraints
            collision = node['collisions'][0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            # Create a child node for each constraint
            for constraint in constraints:
                new_node = {
                    'cost': 0,
                    'constraints': node['constraints'] + [constraint],
                    'paths': node['paths'].copy(),
                    'collisions': []
                }

                # Recompute the path for the agent with the new constraint
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                              self.heuristics[agent], agent, new_node['constraints'])

                if path is None:
                    continue  # Skip if no path is found for this agent

                new_node['paths'][agent] = path

                # If using disjoint splitting, check other agents for violations
                if disjoint and constraint.get('positive'):
                    violating_agents = paths_violate_constraint(constraint, new_node['paths'])
                    # Recalculate paths for all violating agents
                    for violating_agent in violating_agents:
                        path = a_star(self.my_map, self.starts[violating_agent], self.goals[violating_agent],
                                      self.heuristics[violating_agent], violating_agent, new_node['constraints'])
                        if path is None:
                            break  # Skip this child node if any agent cannot find a path
                        new_node['paths'][violating_agent] = path

                    # If any agent cannot satisfy the positive constraint, discard the node
                    if path is None:
                        continue

                # Update cost and collisions for the new node
                new_node['cost'] = get_sum_of_cost(new_node['paths'])
                new_node['collisions'] = detect_collisions(new_node['paths'])
                self.push_node(new_node)

        raise BaseException("No solutions found")

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
        '''
#CBS with DLS
'''class CBSSolver(object):
    """The high-level search of CBS with Depth-Limited Search (DLS)."""

    def __init__(self, my_map, starts, goals, depth_limit=None):
        """
        my_map      - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        depth_limit - (optional) maximum depth for DLS
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.depth_limit = depth_limit

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.num_of_pruned = 0  # Nodes pruned due to depth limit
        self.CPU_time = 0

        self.open_list = []

        # Compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print(f"Generate node {self.num_of_generated} at depth {node.get('depth', 0)}")
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print(f"Expand node {id} at depth {node.get('depth', 0)}")
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """Find paths for all agents with depth-limited search."""

        self.start_time = timer.time()

        # Generate the root node
        root = {
            'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': [],
            'depth': 0  # Track depth of the node
        }
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        while len(self.open_list) > 0:
            node = self.pop_node()

            # Check depth limit
            if self.depth_limit is not None and node['depth'] >= self.depth_limit:
                print(f"Node {self.num_of_expanded} pruned due to depth limit")
                self.num_of_pruned += 1
                continue

            # If no collisions, return the solution
            if not node['collisions']:
                self.print_results(node)
                return node['paths']

            # Handle collisions
            collision = node['collisions'][0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            # Create child nodes for each constraint
            for constraint in constraints:
                child_node = {
                    'cost': 0,
                    'constraints': node['constraints'] + [constraint],
                    'paths': node['paths'].copy(),
                    'collisions': [],
                    'depth': node['depth'] + 1  # Increment depth
                }

                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                              self.heuristics[agent], agent, child_node['constraints'])

                if path is None:
                    continue

                child_node['paths'][agent] = path

                # Handle disjoint splitting
                if disjoint and constraint.get('positive'):
                    violating_agents = paths_violate_constraint(constraint, child_node['paths'])
                    for violating_agent in violating_agents:
                        path = a_star(self.my_map, self.starts[violating_agent], self.goals[violating_agent],
                                      self.heuristics[violating_agent], violating_agent, child_node['constraints'])
                        if path is None:
                            break
                        child_node['paths'][violating_agent] = path
                    if path is None:
                        continue

                child_node['cost'] = get_sum_of_cost(child_node['paths'])
                child_node['collisions'] = detect_collisions(child_node['paths'])
                self.push_node(child_node)

        raise BaseException("No solutions found")

    def print_results(self, node):
        CPU_time = timer.time() - self.start_time
        print("\nFound a solution!")
        print(f"CPU time (s): {CPU_time:.2f}")
        print(f"Sum of costs: {get_sum_of_cost(node['paths'])}")
        print(f"Expanded nodes: {self.num_of_expanded}")
        print(f"Generated nodes: {self.num_of_generated}")
        print(f"Pruned nodes (due to depth limit): {self.num_of_pruned}")
'''
#IDCBS
'''
class CBSSolver(object):
    """The high-level search of CBS with Iterative Deepening."""

    def __init__(self, my_map, starts, goals):
        """
        my_map      - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.num_of_pruned = 0  # Nodes pruned due to depth limit
        self.CPU_time = 0

        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self, disjoint=True):
        """
        Iterative Deepening CBS: Incrementally increases the depth limit and retries the search until a solution is found.
        """
        self.start_time = timer.time()
        depth_limit = 1  # Start with an initial depth limit
        iterations = 0
        total_time = 0

        while True:
            iterations += 1
            print(f"\n*** Iteration {iterations}: Searching with depth limit = {depth_limit} ***")

            # Reinitialize variables for the current iteration
            self.num_of_generated = 0
            self.num_of_expanded = 0
            self.num_of_pruned = 0
            self.open_list = []

            # Attempt to find a solution with the current depth limit
            try:
                solution = self.find_solution_with_depth_limit(depth_limit, disjoint)
                CPU_time = timer.time() - self.start_time
                print(f"\nSolution found at depth limit = {depth_limit} after {iterations} iterations")
                print(f"CPU time (s): {CPU_time:.2f}")
                print(f"Cumulative Expanded nodes: {self.num_of_expanded}")
                print(f"Cumulative Generated nodes: {self.num_of_generated}")
                print(f"Cumulative Pruned nodes: {self.num_of_pruned}")
                return solution
            except BaseException as e:
                print(f"No solution found at depth limit = {depth_limit}. Increasing limit...")
                depth_limit += 1  # Increase depth limit
                total_time = timer.time() - self.start_time

    def find_solution_with_depth_limit(self, depth_limit, disjoint=True):
        """Depth-limited CBS."""
        root = {
            'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': [],
            'depth': 0  # Track depth of the node
        }
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        while len(self.open_list) > 0:
            node = self.pop_node()

            # Check depth limit
            if node['depth'] >= depth_limit:
                print(f"Node {self.num_of_expanded} pruned due to depth limit")
                self.num_of_pruned += 1
                continue

            # If no collisions, return the solution
            if not node['collisions']:
                self.print_results(node)
                return node['paths']

            # Handle collisions
            collision = node['collisions'][0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            # Create child nodes for each constraint
            for constraint in constraints:
                child_node = {
                    'cost': 0,
                    'constraints': node['constraints'] + [constraint],
                    'paths': node['paths'].copy(),
                    'collisions': [],
                    'depth': node['depth'] + 1  # Increment depth
                }

                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                              self.heuristics[agent], agent, child_node['constraints'])

                if path is None:
                    continue

                child_node['paths'][agent] = path

                # Handle disjoint splitting
                if disjoint and constraint.get('positive'):
                    violating_agents = paths_violate_constraint(constraint, child_node['paths'])
                    for violating_agent in violating_agents:
                        path = a_star(self.my_map, self.starts[violating_agent], self.goals[violating_agent],
                                      self.heuristics[violating_agent], violating_agent, child_node['constraints'])
                        if path is None:
                            break
                        child_node['paths'][violating_agent] = path
                    if path is None:
                        continue

                child_node['cost'] = get_sum_of_cost(child_node['paths'])
                child_node['collisions'] = detect_collisions(child_node['paths'])
                self.push_node(child_node)

        raise BaseException("No solutions found within depth limit")

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print(f"Generate node {self.num_of_generated} at depth {node.get('depth', 0)}")
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print(f"Expand node {id} at depth {node.get('depth', 0)}")
        self.num_of_expanded += 1
        return node

    def print_results(self, node):
        print("\nFound a solution!")
        print(f"Sum of costs: {get_sum_of_cost(node['paths'])}")
        print(f"Expanded nodes: {self.num_of_expanded}")
        print(f"Generated nodes: {self.num_of_generated}")
        print(f"Pruned nodes (due to depth limit): {self.num_of_pruned}")
        '''
#IDCBS with LPA*
'''class CBSSolver(object):
    """The high-level search of CBS with Iterative Deepening."""

    def __init__(self, my_map, starts, goals):
        """
        my_map      - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.num_of_pruned = 0  # Nodes pruned due to depth limit
        self.CPU_time = 0

        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self, disjoint=True):
        """
        Iterative Deepening CBS: Incrementally increases the depth limit and retries the search until a solution is found.
        """
        self.start_time = timer.time()
        depth_limit = 1  # Start with an initial depth limit
        iterations = 0
        total_time = 0

        while True:
            iterations += 1
            print(f"\n*** Iteration {iterations}: Searching with depth limit = {depth_limit} ***")

            # Reinitialize variables for the current iteration
            self.num_of_generated = 0
            self.num_of_expanded = 0
            self.num_of_pruned = 0
            self.open_list = []

            # Attempt to find a solution with the current depth limit
            try:
                solution = self.find_solution_with_depth_limit(depth_limit, disjoint)
                CPU_time = timer.time() - self.start_time
                print(f"\nSolution found at depth limit = {depth_limit} after {iterations} iterations")
                print(f"CPU time (s): {CPU_time:.2f}")
                print(f"Cumulative Expanded nodes: {self.num_of_expanded}")
                print(f"Cumulative Generated nodes: {self.num_of_generated}")
                print(f"Cumulative Pruned nodes: {self.num_of_pruned}")
                return solution
            except BaseException as e:
                print(f"No solution found at depth limit = {depth_limit}. Increasing limit...")
                depth_limit += 1  # Increase depth limit
                total_time = timer.time() - self.start_time

    def find_solution_with_depth_limit(self, depth_limit, disjoint=True):
        """Depth-limited CBS."""
        root = {
            'cost': 0,
            'constraints': [],
            'paths': [],
            'collisions': [],
            'depth': 0  # Track depth of the node
        }
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = lpa_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root)

        while len(self.open_list) > 0:
            node = self.pop_node()

            # Check depth limit
            if node['depth'] >= depth_limit:
                print(f"Node {self.num_of_expanded} pruned due to depth limit")
                self.num_of_pruned += 1
                continue

            # If no collisions, return the solution
            if not node['collisions']:
                self.print_results(node)
                return node['paths']

            # Handle collisions
            collision = node['collisions'][0]
            if disjoint:
                constraints = disjoint_splitting(collision)
            else:
                constraints = standard_splitting(collision)

            # Create child nodes for each constraint
            for constraint in constraints:
                child_node = {
                    'cost': 0,
                    'constraints': node['constraints'] + [constraint],
                    'paths': node['paths'].copy(),
                    'collisions': [],
                    'depth': node['depth'] + 1  # Increment depth
                }

                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent],
                              self.heuristics[agent], agent, child_node['constraints'])

                if path is None:
                    continue

                child_node['paths'][agent] = path

                # Handle disjoint splitting
                if disjoint and constraint.get('positive'):
                    violating_agents = paths_violate_constraint(constraint, child_node['paths'])
                    for violating_agent in violating_agents:
                        path = a_star(self.my_map, self.starts[violating_agent], self.goals[violating_agent],
                                      self.heuristics[violating_agent], violating_agent, child_node['constraints'])
                        if path is None:
                            break
                        child_node['paths'][violating_agent] = path
                    if path is None:
                        continue

                child_node['cost'] = get_sum_of_cost(child_node['paths'])
                child_node['collisions'] = detect_collisions(child_node['paths'])
                self.push_node(child_node)

        raise BaseException("No solutions found within depth limit")

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        print(f"Generate node {self.num_of_generated} at depth {node.get('depth', 0)}")
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print(f"Expand node {id} at depth {node.get('depth', 0)}")
        self.num_of_expanded += 1
        return node

    def print_results(self, node):
        print("\nFound a solution!")
        print(f"Sum of costs: {get_sum_of_cost(node['paths'])}")
        print(f"Expanded nodes: {self.num_of_expanded}")
        print(f"Generated nodes: {self.num_of_generated}")
        print(f"Pruned nodes (due to depth limit): {self.num_of_pruned}")
'''
#IDCBS with LPA*, MDD and CAT
class CBSSolver:
    """CBS Solver with Multi-Value Decision Diagrams (MDD) and Conflict Avoidance Table (CAT)."""

    def __init__(self, my_map, starts, goals):
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.open_list = []

        # Heuristics for low-level search
        self.heuristics = [compute_heuristics(my_map, goal) for goal in self.goals]

        # Global Conflict Avoidance Table
        self.global_cat = {}
        self.mdds = {}  # Store MDDs for all agents

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node

    def build_mdd(self, agent, constraints):
        """
        Build an MDD for the given agent under the specified constraints.
        """
        print(f"Building MDD for agent {agent}...")
        # Implement forward search to depth = optimal cost
        # Perform backward pruning to get the MDD
        # Store in self.mdds[agent]
        # Placeholder:
        self.mdds[agent] = []  # Replace this with actual MDD building logic.

    def update_cat(self, agent, path, remove=False):
        """
        Update the global CAT table for an agent's path.
        """
        for t, loc in enumerate(path):
            if remove:
                self.global_cat.pop((loc, t), None)
            else:
                self.global_cat[(loc, t)] = agent

    def find_solution(self, disjoint=True):
        """Finds paths for all agents using CBS with MDD and CAT."""
        # Root node initialization
        root = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}
        for i in range(self.num_of_agents):
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, root['constraints'])
            if path is None:
                raise BaseException(f"No solutions for agent {i}")
            root['paths'].append(path)
            self.update_cat(i, path)  # Update CAT for the initial paths

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = self.detect_collisions(root['paths'])
        self.push_node(root)

        while self.open_list:
            node = self.pop_node()

            # If no collisions, return the solution
            if not node['collisions']:
                self.print_results(node)
                return node['paths']

            # Handle collisions
            collision = node['collisions'][0]
            constraints = self.standard_splitting(collision)

            for constraint in constraints:
                new_node = {'constraints': node['constraints'] + [constraint], 'paths': node['paths'][:],
                            'collisions': []}

                agent = constraint['agent']
                # Remove old path from CAT
                self.update_cat(agent, new_node['paths'][agent], remove=True)

                # Build or reuse MDD for the agent
                if agent not in self.mdds:
                    self.build_mdd(agent, new_node['constraints'])

                # Perform pathfinding with updated constraints
                path = lpa_star(self.my_map, self.starts[agent], self.goals[agent], self.heuristics[agent], agent,
                              new_node['constraints'])

                if path is None:
                    continue  # No valid path, skip node

                new_node['paths'][agent] = path
                self.update_cat(agent, path)  # Update CAT with new path

                # Recalculate cost and collisions
                new_node['cost'] = get_sum_of_cost(new_node['paths'])
                new_node['collisions'] = self.detect_collisions(new_node['paths'])
                self.push_node(new_node)

        raise BaseException("No solutions found")

    def detect_collisions(self, paths):
        """Detect collisions among all agents' paths."""
        collisions = []
        for i in range(self.num_of_agents):
            for j in range(i + 1, self.num_of_agents):
                collision = self.detect_collision(paths[i], paths[j])
                if collision:
                    collisions.append({'a1': i, 'a2': j, 'loc': collision['loc'], 'timestep': collision['timestep']})
        return collisions

    def detect_collision(self, path1, path2):
        """Detect the first collision between two paths."""
        max_t = max(len(path1), len(path2))
        for t in range(max_t):
            loc1 = get_location(path1, t)
            loc2 = get_location(path2, t)
            if loc1 == loc2:  # Vertex collision
                return {'loc': [loc1], 'timestep': t}
            if t > 0 and get_location(path1, t - 1) == loc2 and get_location(path2, t - 1) == loc1:  # Edge collision
                return {'loc': [get_location(path1, t - 1), loc1], 'timestep': t}
        return None

    def standard_splitting(self, collision):
        """Generate constraints to resolve a collision."""
        a1, a2, loc, t = collision['a1'], collision['a2'], collision['loc'], collision['timestep']
        if len(loc) == 1:  # Vertex collision
            return [{'agent': a1, 'loc': loc, 'timestep': t}, {'agent': a2, 'loc': loc, 'timestep': t}]
        else:  # Edge collision
            return [{'agent': a1, 'loc': loc, 'timestep': t}, {'agent': a2, 'loc': loc[::-1], 'timestep': t}]

    def print_results(self, node):
        print("Found a solution!")
        print("Sum of costs:", get_sum_of_cost(node['paths']))
        print("Expanded nodes:", self.num_of_expanded)
        print("Generated nodes:", self.num_of_generated)



