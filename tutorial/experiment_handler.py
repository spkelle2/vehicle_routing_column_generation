"""
Module to run experiments comparing exact and heuristic VRP solutions after a
given amount of time. Either runs them in series or generates .pbs files so
they can be run on a server cluster in parallel
"""

import os
import re
import shutil
import sys

from tutorial.run_one_experiment import run_one_experiment


def handle_all_experiments(mode: str, sln_directory: str, run_time=None) -> None:
    """ Run both the exact and heuristic VRP methods to compare the quality of
    their solutions for a given amount of time

    :param mode: either 'run' or 'make_pbs_files'. The former runs in series
    and the latter generates .pbs files to run in parallel
    :param sln_directory: where to save the solutions
    :param run_time: overrides the max_solve_time parameter for convenience
    :return: None
    """
    assert mode in ['run', 'make_pbs_files']

    input_root_pth = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'sample_inputs')
    sln_root_pth = os.path.join(os.path.dirname(os.path.realpath(__file__)), sln_directory)
    pattern = re.compile("input_(\d+)")

    # delete solution root folder if it exists and recreate it
    shutil.rmtree(sln_root_pth, ignore_errors=True)

    for i, input_dir in enumerate(sorted(os.listdir(input_root_pth))):
        orders = pattern.match(input_dir).group(1)
        print(f'test {i + 1}: {orders} orders')
        input_pth = os.path.join(input_root_pth, input_dir)

        if mode == 'run':
            run_one_experiment(sln_root_pth, orders, input_pth, run_time)
        else:
            make_pbs_file(sln_root_pth, orders, input_pth, run_time)


def make_pbs_file(sln_root_pth, orders, input_pth, run_time):
    """ make a .pbs file to run a single experiment comparing both algorithms'
    solutions at a specific run time for a given number of orders

    :param sln_root_pth: where to save solutions
    :param orders: number of orders in this problem instance
    :param input_pth: where the data set for this instance can be found
    :param run_time: max run time to cut off solve
    :return:
    """

    assert run_time is not None, 'run_time cannot be None for pbs file generation'

    txt = f"""#PBS -N vrp_{orders}_{run_time}
#PBS -e /home/sek519/vehicle_routing_column_generation/tutorial/vrp_orders_{orders}_time_{run_time}.err
#PBS -o /home/sek519/vehicle_routing_column_generation/tutorial/vrp_orders_{orders}_time_{run_time}.out
#PBS -l ncpus=4,mem=15gb,vmem=15gb,pmem=15gb
#PBS -q 'medium'

cd /home/sek519/vehicle_routing_column_generation/tutorial
source /home/sek519/miniconda/bin/activate
conda activate col_gen
python run_one_experiment.py {sln_root_pth} {orders} {input_pth} {run_time}"""
    text_file = open(f"vrp_orders_{orders}_time_{run_time}.pbs", "w")
    text_file.write(txt)
    text_file.close()


if __name__ == '__main__':

    # call with python experiment_handler.py <run/make_pbs_files> sample_solutions_<run_time> <run_time>
    handle_all_experiments(*sys.argv[1:])
