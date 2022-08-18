import os
import sys

from vehicle_routing_column_generation.algorithms import ExactVRP, HeuristicVRP


def run_one_experiment(sln_root_pth: str, orders: str, input_pth, run_time=None) -> None:
    """ Run both the exact and heuristic VRP methods to compare the quality of
    their solutions for a given amount of time

    :param sln_directory: where to save the solutions
    :param run_time: overrides the max_solve_time parameter for convenience
    :return: None
    """

    # if called directly for the first time, make the solution root directory
    if not os.path.exists(sln_root_pth):
        os.mkdir(sln_root_pth)

    # run heuristic methods
    heuristic_sln_pth = os.path.join(sln_root_pth, f'solution_{orders}_heuristic')
    heuristic_vrp = HeuristicVRP(input_pth=input_pth, solution_pth=heuristic_sln_pth)
    if run_time:
        heuristic_vrp.parameters['max_solve_time'] = int(run_time)
    heuristic_vrp.solve()

    # run exact method
    exact_sln_pth = os.path.join(sln_root_pth, f'solution_{orders}_exact')
    exact_vrp = ExactVRP(input_pth=input_pth, solution_pth=exact_sln_pth)
    if run_time:
        exact_vrp.parameters['max_solve_time'] = int(run_time)
    exact_vrp.solve()


if __name__ == '__main__':

    # call with python run_one_experiment.py <sln_root_pth> <orders> <input_pth> <run_time>
    run_one_experiment(*sys.argv[1:])
