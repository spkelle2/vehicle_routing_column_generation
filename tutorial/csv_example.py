"""
An example of how to run the VRP classes from csv files
"""

import os

from vehicle_routing_column_generation.algorithms import ExactVRP, HeuristicVRP


def main():
    input_pth = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sample_inputs', 'input_10')
    sln_pth = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sample_solution')

    exact_vrp = ExactVRP(input_pth=input_pth, solution_pth=sln_pth)
    exact_vrp.solve()

    heuristic_vrp = HeuristicVRP(input_pth=input_pth, solution_pth=sln_pth)
    heuristic_vrp.solve()

    print()


if __name__ == '__main__':
    main()
