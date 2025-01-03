#!/usr/bin/python
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import time


SOLVER = "CBS"

def print_mapf_instance(my_map, starts, goals):
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)


def import_mapf_instance(filename):
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)
    # #agents lines with the start/goal positions
    starts = []
    goals = []
    for a in range(num_agents):
        line = f.readline()
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    return my_map, starts, goals


'''
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()

    # CSV file to store results
    result_file = open("results.csv", "w", buffering=1)
    result_file.write("Instance,Solver,Disjoint,Cost,CPU Time,Expanded Nodes,Generated Nodes\n")

    for file in sorted(glob.glob(args.instance)):
        print(f"***Import an instance from {file}***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        solver_name = args.solver
        disjoint = args.disjoint
        start_time = time.time()  # Start timing

        # Select the solver
        if solver_name == "CBS":
            print("***Run CBS***")
            solver = CBSSolver(my_map, starts, goals)
            paths = solver.find_solution(disjoint)
        elif solver_name == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif solver_name == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        end_time = time.time()  # End timing

        cost = get_sum_of_cost(paths)
        cpu_time = end_time - start_time
        expanded_nodes = solver.num_of_expanded if hasattr(solver, 'num_of_expanded') else 'N/A'
        generated_nodes = solver.num_of_generated if hasattr(solver, 'num_of_generated') else 'N/A'

        result_file.write(f"{file},{solver_name},{disjoint},{cost},{cpu_time},{expanded_nodes},{generated_nodes}\n")

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            animation.show()

    result_file.close()
'''
if __name__ == '__main__':
    import os

    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()

    # CSV file to store results
    result_file_mode = "a"  # Append mode
    result_file = open("results.csv", result_file_mode, buffering=1)

    # Write headers only if the file is empty
    if not os.path.isfile("results.csv") or os.stat("results.csv").st_size == 0:
        result_file.write("Instance,Solver,Disjoint,Cost,CPU Time,Expanded Nodes,Generated Nodes\n")

    for file in sorted(glob.glob(args.instance)):
        print(f"***Import an instance from {file}***")
        my_map, starts, goals = import_mapf_instance(file)
        print_mapf_instance(my_map, starts, goals)

        solver_name = args.solver
        disjoint = args.disjoint
        start_time = time.time()  # Start timing

        # Select the solver
        if solver_name == "CBS":
            print("***Run CBS***")
            solver = CBSSolver(my_map, starts, goals)
            paths = solver.find_solution(disjoint)
        elif solver_name == "Independent":
            print("***Run Independent***")
            solver = IndependentSolver(my_map, starts, goals)
            paths = solver.find_solution()
        elif solver_name == "Prioritized":
            print("***Run Prioritized***")
            solver = PrioritizedPlanningSolver(my_map, starts, goals)
            paths = solver.find_solution()
        else:
            raise RuntimeError("Unknown solver!")

        end_time = time.time()  # End timing

        cost = get_sum_of_cost(paths)
        cpu_time = end_time - start_time
        expanded_nodes = solver.num_of_expanded if hasattr(solver, 'num_of_expanded') else 'N/A'
        generated_nodes = solver.num_of_generated if hasattr(solver, 'num_of_generated') else 'N/A'

        result_file.write(f"{file},{solver_name},{disjoint},{cost},{cpu_time},{expanded_nodes},{generated_nodes}\n")

        if not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            animation.show()

    result_file.close()
