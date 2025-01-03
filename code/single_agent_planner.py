import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.
    # constraint_table = {}
    # for constraint in constraints:
    #     if constraint['agent'] == agent:
    #         timestep = constraint['timestep']
    #         loc = constraint['loc']
    #         if timestep not in constraint_table:
    #             constraint_table[timestep] = []
    #         constraint_table[timestep].append(loc)
    # return constraint_table
    constraint_table = []
    for constraint in constraints:
        if constraint['agent'] == agent or constraint['agent'] == -1:
            # Add the positive attribute to the constraint
            constraint_table.append({
                'loc': constraint['loc'],
                'timestep': constraint['timestep'],
                'positive': constraint.get('positive', False)  # Defaults to False if not specified
            })
    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.
    # if next_time in constraint_table:
    #     if [next_loc] in constraint_table[next_time]:
    #         return True
    # return False
    # if next_time in constraint_table:
    #     # Check for vertex constraint
    #     if [next_loc] in constraint_table[next_time]:
    #         return True
    #
    #     # Check for edge constraint
    #     if [curr_loc, next_loc] in constraint_table[next_time]:
    #         return True
    #
    # return False
    for constraint in constraint_table:
        if constraint['timestep'] != next_time:
            continue

        # Handle vertex constraints
        if len(constraint['loc']) == 1:
            if constraint['loc'][0] == next_loc:
                if constraint['positive']:
                    # Positive vertex constraint: must be at next_loc at next_time
                    if curr_loc != next_loc:
                        return True  # Constraint violated: path does not satisfy the positive constraint
                else:
                    # Negative vertex constraint: cannot be at next_loc at next_time
                    return True  # Constraint violated

        # Handle edge constraints
        elif len(constraint['loc']) == 2:
            if constraint['loc'] == [curr_loc, next_loc]:
                if constraint['positive']:
                    # Positive edge constraint: must move from curr_loc to next_loc at next_time
                    return False  # Path is not constrained, meets the positive constraint
                else:
                    # Negative edge constraint: cannot move from curr_loc to next_loc
                    return True  # Constraint violated

    return False  # No constraints violated

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def update_cat(cat, path, agent_id, remove=False):
    """
    Update the Conflict Avoidance Table (CAT) by adding or removing an agent's path.
    """
    for t, loc in enumerate(path):
        if remove:
            cat.pop((loc, t), None)
        else:
            cat[(loc, t)] = agent_id

def build_mdd(my_map, start, goal, cost, constraints):
    """
    Build a Multi-Value Decision Diagram (MDD) for a given agent.
    """
    print(f"Building MDD with cost {cost} for agent at start {start} to goal {goal}")
    forward_nodes = {}
    open_list = []

    # Forward A* search
    root = {'loc': start, 'time': 0, 'cost': 0, 'parent': None}
    heapq.heappush(open_list, (0, root))
    forward_nodes[(start, 0)] = root

    while open_list:
        _, curr = heapq.heappop(open_list)
        if curr['cost'] >= cost:  # Do not expand beyond the cost limit
            continue

        for dir in range(4):
            child_loc = move(curr['loc'], dir)
            child_time = curr['time'] + 1

            # Check constraints
            if is_constrained(curr['loc'], child_loc, child_time, constraints):
                continue

            if 0 <= child_loc[0] < len(my_map) and 0 <= child_loc[1] < len(my_map[0]) and not my_map[child_loc[0]][child_loc[1]]:
                new_node = {'loc': child_loc, 'time': child_time, 'cost': curr['cost'] + 1, 'parent': curr}
                forward_nodes[(child_loc, child_time)] = new_node
                heapq.heappush(open_list, (new_node['cost'], new_node))

    # Backward pruning
    mdd = {}
    goal_node = [(loc, time) for (loc, time), node in forward_nodes.items() if loc == goal and node['cost'] == cost]

    if not goal_node:
        return None  # No MDD possible

    open_list = goal_node
    while open_list:
        curr_loc, curr_time = open_list.pop()
        mdd.setdefault(curr_time, []).append(curr_loc)
        for dir in range(4):
            parent_loc = move(curr_loc, (dir + 2) % 4)  # Reverse direction
            parent_time = curr_time - 1

            if (parent_loc, parent_time) in forward_nodes:
                open_list.append((parent_loc, parent_time))

    return mdd
def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()

    constraint_table = build_constraint_table(constraints, agent)

    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time'])] = root  # Use (cell, time) as key

    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints

        if curr['loc'] == goal_loc:
            if not is_constrained(curr['loc'], curr['loc'], curr['time'], constraint_table):
                # Allow agent to stay at the goal without constraints
                if all(not is_constrained(goal_loc, goal_loc, t, constraint_table)
                       for t in range(curr['time'], curr['time'] + 5)):  # Allow up to 5 time steps
                    return get_path(curr)

        # Generate children by moving in the four directions
        for dir in range(4):
            child_loc = move(curr['loc'], dir)

            # Check bounds and obstacles
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) or \
                    child_loc[1] < 0 or child_loc[1] >= len(my_map[0]) or \
                    my_map[child_loc[0]][child_loc[1]]:
                continue

            # Create the child node with incremented time
            child = {
                'loc': child_loc,
                'g_val': curr['g_val'] + 1,
                'h_val': h_values[child_loc],
                'parent': curr,
                'time': curr['time'] + 1
            }

            # Check for edge constraint violations
            if is_constrained(curr['loc'], child_loc, child['time'], constraint_table):
                continue

            # Add child node if not already visited at this time step
            if (child['loc'], child['time']) not in closed_list:
                closed_list[(child['loc'], child['time'])] = child
                push_node(open_list, child)

            # Add a wait action (agent stays at current location)
        wait_child = {
            'loc': curr['loc'],
            'g_val': curr['g_val'] + 1,
            'h_val': curr['h_val'],
            'parent': curr,
            'time': curr['time'] + 1
        }

        # Check if waiting violates constraints
        if not is_constrained(curr['loc'], curr['loc'], wait_child['time'], constraint_table):
            if (wait_child['loc'], wait_child['time']) not in closed_list:
                closed_list[(wait_child['loc'], wait_child['time'])] = wait_child
                push_node(open_list, wait_child)


    return None  # Failed to find solutions

#LPA*
class LPAStar:
    def __init__(self, my_map, start_loc, goal_loc, h_values, agent, constraints):
        self.my_map = my_map
        self.start_loc = start_loc
        self.goal_loc = goal_loc
        self.h_values = h_values
        self.agent = agent
        self.constraints = constraints
        self.g_scores = {}  # Cost from start to a node
        self.rhs_scores = {}  # "One-step lookahead" cost to a node
        self.open_list = []
        self.closed_set = set()

    def initialize(self):
        """Initialize the scores and priority queue."""
        self.g_scores[self.start_loc] = float('inf')
        self.rhs_scores[self.start_loc] = 0
        self.g_scores[self.goal_loc] = float('inf')
        self.rhs_scores[self.goal_loc] = float('inf')
        self.push_node(self.start_loc)

    def heuristic(self, loc):
        """Return the heuristic (h-value) for a given location."""
        return self.h_values.get(loc, float('inf'))

    def push_node(self, loc):
        """Push a node into the priority queue."""
        g = self.g_scores.get(loc, float('inf'))
        rhs = self.rhs_scores.get(loc, float('inf'))
        priority = min(g, rhs) + self.heuristic(loc)
        heapq.heappush(self.open_list, (priority, loc))

    def update_node(self, loc):
        """Update a node and push it to the queue if necessary."""
        if loc in self.closed_set:
            return
        g = self.g_scores.get(loc, float('inf'))
        rhs = self.rhs_scores.get(loc, float('inf'))
        if g != rhs:
            self.push_node(loc)

    def compute_shortest_path(self):
        """Incrementally compute the shortest path."""
        while self.open_list:
            _, current = heapq.heappop(self.open_list)
            self.closed_set.add(current)

            g = self.g_scores.get(current, float('inf'))
            rhs = self.rhs_scores.get(current, float('inf'))

            if g > rhs:
                self.g_scores[current] = rhs
                for neighbor in self.get_neighbors(current):
                    self.update_rhs(neighbor)
                    self.update_node(neighbor)
            else:
                self.g_scores[current] = float('inf')
                for neighbor in self.get_neighbors(current):
                    self.update_rhs(neighbor)
                    self.update_node(neighbor)

    def update_rhs(self, loc):
        """Update the RHS value for a node."""
        if loc == self.start_loc:
            return  # Start node RHS is always 0
        neighbors = self.get_neighbors(loc)
        rhs = min(self.g_scores.get(n, float('inf')) + 1 for n in neighbors)  # Cost + edge weight (1)
        self.rhs_scores[loc] = rhs

    def get_neighbors(self, loc):
        """Get valid neighbors for a location."""
        neighbors = []
        for dir in range(4):  # Up, right, down, left
            new_loc = move(loc, dir)
            if 0 <= new_loc[0] < len(self.my_map) and \
               0 <= new_loc[1] < len(self.my_map[0]) and \
               not self.my_map[new_loc[0]][new_loc[1]]:  # Check bounds and obstacles
                neighbors.append(new_loc)
        return neighbors

    def get_path(self):
        """Extract the shortest path to the goal."""
        path = []
        current = self.goal_loc
        while current != self.start_loc:
            path.append(current)
            current = min(self.get_neighbors(current), key=lambda n: self.g_scores.get(n, float('inf')))
        path.append(self.start_loc)
        path.reverse()
        return path


def lpa_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """Wrapper for LPA* to match A*'s interface."""
    lpa = LPAStar(my_map, start_loc, goal_loc, h_values, agent, constraints)
    lpa.initialize()
    lpa.compute_shortest_path()
    return lpa.get_path()