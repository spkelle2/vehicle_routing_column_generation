import os
import re
import shutil
import sys

from vehicle_routing_column_generation.algorithms import ExactVRP, HeuristicVRP


def main(sln_directory: str, run_time=None) -> None:
    """ Run both the exact and heuristic VRP methods to compare the quality of
    their solutions for a given amount of time

    :param sln_directory: where to save the solutions
    :param run_time: overrides the max_solve_time parameter for convenience
    :return: None
    """

    if run_time:
        run_time = int(run_time)
    input_root_pth = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sample_inputs')
    sln_root_pth = os.path.join(os.path.dirname(os.path.realpath(__file__)), sln_directory)
    pattern = re.compile("input_(\d+)")

    # delete solution root folder if it exists and recreate it
    shutil.rmtree(sln_root_pth, ignore_errors=True)
    os.mkdir(sln_root_pth)

    for i, input_dir in enumerate(sorted(os.listdir(input_root_pth))):
        orders = pattern.match(input_dir).group(1)
        print(f'running test {i+1}: {orders} orders')
        input_pth = os.path.join(input_root_pth, input_dir)

        # run exact method
        exact_sln_pth = os.path.join(sln_root_pth, f'solution_{orders}_exact')
        exact_vrp = ExactVRP(input_pth=input_pth, solution_pth=exact_sln_pth)
        if run_time:
            exact_vrp.parameters['max_solve_time'] = run_time
        exact_vrp.solve()

        # run heuristic methods
        heuristic_sln_pth = os.path.join(sln_root_pth, f'solution_{orders}_heuristic')
        heuristic_vrp = HeuristicVRP(input_pth=input_pth, solution_pth=heuristic_sln_pth)
        if run_time:
            heuristic_vrp.parameters['max_solve_time'] = run_time
        heuristic_vrp.solve()


if __name__ == '__main__':

    # call with python run_sample_inputs.py sample_solutions_<run_time> <run_time>
    main(*sys.argv[1:])
