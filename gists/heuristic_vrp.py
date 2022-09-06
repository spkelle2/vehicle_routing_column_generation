"""
Constructs minimal example to run column generation heuristic for vehicle routing.
* Defines input and output data requirements
* Builds in-memory dataset
* Defines base class, VRP, for data input and validation
* Defines subclass, HeuristicVRP, for solving VRP with column generation
"""

import gurobipy as gu
import os
from ticdat import TicDatFactory
from ticdat.jsontd import make_json_dict
import time
from typing import Tuple, Any, Union


# ----------------- Define Input and Output Data Requirements ------------------
# label column headers and set primary key constraints
input_schema = TicDatFactory(
    arc=[['start_idx', 'end_idx'], ['travel_time', 'cost']],
    node=[['idx'], ['name', 'type', 'lat', 'long', 'open', 'close']],
    order=[['node_idx'], ['weight']],
    parameters=[['key'], ['value']]  # truck capacity
)

# set type constraints (pks default to strings. other cols default to floats)
input_schema.set_data_type('arc', 'start_idx', must_be_int=True)
input_schema.set_data_type('arc', 'end_idx', must_be_int=True)
input_schema.set_data_type('node', 'idx', must_be_int=True)
input_schema.set_data_type('node', 'name', number_allowed=False, strings_allowed="*", nullable=True)
input_schema.set_data_type('node', 'type', number_allowed=False, strings_allowed=('depot', 'customer'))
input_schema.set_data_type('node', 'lat', min=-90, max=90, inclusive_max=True)
input_schema.set_data_type('node', 'long', min=-180, max=180, inclusive_max=True)
input_schema.set_data_type('node', 'open', max=24, inclusive_max=True)
input_schema.set_data_type('node', 'close', max=24, inclusive_max=True)
input_schema.set_data_type('order', 'node_idx', must_be_int=True)

# set foreign key constraints (all node indices must be an index of the node table)
input_schema.add_foreign_key('arc', 'node', ['start_idx', 'idx'])
input_schema.add_foreign_key('arc', 'node', ['end_idx', 'idx'])
input_schema.add_foreign_key('order', 'node', ['node_idx', 'idx'])

# set check constraints (all locations close no earlier than they open)
input_schema.add_data_row_predicate('node', predicate_name="open_close_check",
    predicate=lambda row: row['open'] <= row['close'])

# set parameter constraints
input_schema.add_parameter("truck_capacity", 40000)
input_schema.add_parameter("max_solve_time", 60)
input_schema.add_parameter("solutions_per_pricing_problem", "number_customers",
                           strings_allowed=("number_customers",))
input_schema.add_parameter("pricing_problem_mip_gap", .1, max=1)
input_schema.add_parameter("pricing_problem_time_limit", 1)
input_schema.add_parameter("min_column_generation_progress", .001, max=1)
input_schema.add_parameter("column_generation_solve_ratio", .9, max=1)

# solution tables
solution_schema = TicDatFactory(
    summary=[['key'], ['value']],  # routes and cost
    route=[['idx', 'stop'], ['node_idx', 'arrival']]
)

solution_schema.set_data_type('route', 'idx', must_be_int=True)
solution_schema.set_data_type('route', 'stop', must_be_int=True)


# --------------- Define Static Input Data Set for Example Run -----------------
toy_input = {
    'arc': {
        (0, 1): {'travel_time': 2.3639163739810654, 'cost': 618.1958186990532},
        (1, 0): {'travel_time': 2.3639163739810654, 'cost': 118.19581869905328},
        (0, 2): {'travel_time': 1.5544182164530995, 'cost': 577.720910822655},
        (2, 0): {'travel_time': 1.5544182164530995, 'cost': 77.72091082265497},
        (1, 2): {'travel_time': 0.853048419193608, 'cost': 42.6524209596804},
        (2, 1): {'travel_time': 0.853048419193608, 'cost': 42.6524209596804}
    },
    'node': {
        0: {'name': 'depot', 'type': 'depot', 'lat': 39.91, 'long': -76.5, 'open': 0, 'close': 24},
        1: {'name': 'customer 1', 'type': 'customer', 'lat': 39.91, 'long': -74.61, 'open': 13, 'close': 21},
        2: {'name': 'customer 2', 'type': 'customer', 'lat': 39.78, 'long': -75.27, 'open': 7, 'close': 15}
    },
    'order': {
        1: {'weight': 13084},
        2: {'weight': 8078}
    },
    'parameters': {
        'truck_capacity': {'value': 40000},
        'fleet_size': {'value': 2},
        'max_solve_time': {'value': 60},
        'exact_vrp_mip_gap': {'value': .01},
        'solutions_per_pricing_problem': {'value': 'number_customers'},
        'pricing_problem_mip_gap': {'value': .1},
        'pricing_problem_time_limit': {'value': 1},
        'min_column_generation_progress': {'value': .001},
        'column_generation_solve_ratio': {'value': .9}
    }
}


class VRP:

    def __init__(self, input_pth: str = None, solution_pth: str = None,
                 input_dict: dict[str, dict[str, Any]] = None, solution_dict=False):
        """Base constructor for VRP. Checks the validity of the input data and
        solution path. Assigns common attributes.

        :param input_pth: The location of the directory of CSV's storing input data
        :param solution_pth: The location of the directory of CSV's where solution data written
        :param input_dict: Dictionary storing input data
        :param solution_dict: Whether or not to return solution data as a dictionary
        """
        assert solution_pth or solution_dict, 'must specify where to save solution'
        if solution_pth:
            assert os.path.isdir(os.path.dirname(solution_pth)), \
                'the parent directory of sln_pth must exist'

        # assign base class attributes
        self.solution_pth = solution_pth
        dat = self._data_checks(input_pth, input_dict)
        self.dat = dat
        self.parameters = input_schema.create_full_parameters_dict(dat)
        self.depot_idx = [i for i, f in self.dat.node.items() if f['type'] == 'depot'].pop()
        self.fleet = list(range(self.parameters['fleet_size']))
        self.M = max(dat.node[i]['close'] + f['travel_time'] - dat.node[i]['open']
                     for (i, j), f in dat.arc.items()) + 1

    @staticmethod
    def _data_checks(input_pth: str = None, input_dict: dict[str, dict[str, Any]] = None) -> \
            input_schema.TicDat:
        """ Read in data used for solving this VRP instance. Confirms the data
        match the schema defined in schemas.input_schema. Check that the data
        match the remaining assumptions of our VRP models.

        :param input_pth: The directory containing the CSV's of input data for this VRP instance
        :param input_dict: Dictionary storing input data
        :return: a TicDat containing our input data
        """
        assert input_pth or input_dict, 'must specify an input'
        if input_pth:
            assert os.path.isdir(input_pth), 'input_pth must be a valid directory'

        # read in and do basic data checks
        if input_pth:
            pk_fails = input_schema.csv.find_duplicates(input_pth)
            dat = input_schema.csv.create_tic_dat(input_pth)
        else:
            pk_fails = {}
            dat = input_schema.TicDat(**input_dict)
        fk_fails = input_schema.find_foreign_key_failures(dat)
        type_fails = input_schema.find_data_type_failures(dat)
        check_fails = input_schema.find_data_row_failures(dat)
        assert not (pk_fails and fk_fails and type_fails and check_fails), \
            "The following data failures were found: \n" \
            f"Primary Key: {len(pk_fails)}, Foreign Key: {len(fk_fails)}, " \
            f"Type Constraint: {len(type_fails)}, Check Constraint: {len(check_fails)}"

        # advanced data checks
        p = input_schema.create_full_parameters_dict(dat)
        assert len([f for f in dat.node.values() if f['type'] == 'depot']) == 1, \
            "There can only be one depot amongst the nodes"
        assert not [f for f in dat.order.values() if f['weight'] > p['truck_capacity']], \
            "No order can weigh more than truck capacity"
        assert len(dat.node) - len(dat.order) == 1, \
            "There should be exactly one order for each customer"

        return dat


class HeuristicVRP(VRP):

    def __init__(self, **kwargs):
        """Constructor for heuristic VRP, which uses column generation to create
        a set of covering routes before selecting a good subset of them. Builds
        master and pricing models in gurobi.

        :param **kwargs: keyword arguments to pass onto base constructor
        """
        init_start = time.time()
        super().__init__(**kwargs)
        self.mdl, self.z, self.c, self.route = self._create_master_problem()
        self.sub_mdl, self.x, self.s = self._create_subproblem()
        self.init_time = time.time() - init_start

    def _create_master_problem(self) -> \
            Tuple[gu.Model, dict[int, gu.Var], dict[int, gu.Var], dict[int, dict[int, dict[str, Any]]]]:
        """ Create the gurobi model for the restricted set covering problem. Will
        eventually select a collection of routes that fulfill all customer orders.
        Begins as an LP to yield dual values for pricing problem. Once all desired
        routes (columns) have been added, variables can be made binary to select
        the most cost efficient subset.

        :return: mdl, z, c, and route, which are respectively the master problem
        gurobi model, dictionary of variables representing which routes are selected,
        a dictionary of the model's constraints, and a dictionary of the stop order
        and arrival times for each route represented in the model.
        """
        # make model
        mdl = gu.Model("heuristic_vrp")

        # map a route index to the customer node for each singleton route
        singleton = dict(enumerate(self.dat.order.keys()))

        # order of stops in the route represented by each variable
        # initialize to be the set of singleton routes (i.e. travel from depot to one customer and back)
        route = {route_idx: {
            0: {'node_idx': self.depot_idx, 'arrival': 0},
            1: {'node_idx': j, 'arrival': max(self.dat.arc[self.depot_idx, j]['travel_time'],
                                              self.dat.node[j]['open'])},
            2: {'node_idx': self.depot_idx, 'arrival': 24}
        } for route_idx, j in singleton.items()}

        # create variables and set objective
        # z_i - if route i is chosen - begins relaxed for column generation
        # initialize variables to represent the set of singleton routes created above
        z = {route_idx: mdl.addVar(obj=self.dat.arc[self.depot_idx, j]['cost'] +
                                   self.dat.arc[j, self.depot_idx]['cost'], name=f'z_{route_idx}')
             for route_idx, j in singleton.items()}

        # set constraints
        # 8) each customer must be visited by a route (i.e. have delivery demand met)
        # since starting routes are singletons, each must be selected to cover
        c = {j: mdl.addConstr(z[route_idx] >= 1, name=f'c_{j}') for route_idx, j
             in singleton.items()}

        return mdl, z, c, route

    def _create_subproblem(self) -> \
            Tuple[gu.Model, dict[Tuple[int, int], gu.Var], dict[int, gu.Var]]:
        """ Create the gurobi model for the pricing problem. Formulation adapted
        from https://how-to.aimms.com/Articles/332/332-Formulation-CVRP.html
        and https://how-to.aimms.com/Articles/332/332-Time-Windows.html. Since this
        model generates a single route, indexing over all trucks and requiring
        that all customers are visited are excepted.

        :return: sub_mdl, x, and s, which are respectively the gurobi model, a
        dictionary of variables representing arcs traveled, and a dictionary
        of variables representing arrival times at each customer
        """
        # make model
        sub_mdl = gu.Model("vrp_subproblem")

        # save extra solutions to generate many routes per pricing problem solve
        sub_mdl.setParam(
            "PoolSolutions", len(self.dat.order) if
            self.parameters['solutions_per_pricing_problem'] == 'number_customers'
            else int(self.parameters['solutions_per_pricing_problem'])
        )

        # force early termination of pricing problem so we can solve it repeatedly
        # in fixed time period. tweak these parameters to find the right trade-off
        # between quantity and quality of columns generated
        sub_mdl.setParam("MIPGap", self.parameters['pricing_problem_mip_gap'])
        sub_mdl.setParam("TimeLimit", self.parameters['pricing_problem_mip_gap'])

        # create variables
        # x_i_j if this route travels from node i to node j
        x = {(i, j): sub_mdl.addVar(vtype=gu.GRB.BINARY, name=f'x_{i}_{j}')
             for i in self.dat.node for j in self.dat.node if i != j}
        # s_i time when service begins at node i
        s = {i: sub_mdl.addVar(lb=f['open'], ub=f['close'], name=f's_{i}')
             for i, f in self.dat.node.items()}

        # set constraints
        # 9) Any node j entered by this route must be left
        for j in self.dat.node:
            sub_mdl.addConstr(
                gu.quicksum(x[i, j] for i in self.dat.node if i != j) -
                gu.quicksum(x[j, h] for h in self.dat.node if j != h) == 0,
                name=f"flow_conserve_{j}"
            )
        # 10) The route leaves the depot at most once
        sub_mdl.addConstr(
            gu.quicksum(x[self.depot_idx, j] for j in self.dat.order) <= 1,
            name=f"include_depot"
        )
        # 11) Route stays within capacity
        sub_mdl.addConstr(
            gu.quicksum(gu.quicksum(f['weight'] * x[i, j] for j, f in self.dat.order.items()
                                    if i != j) for i in self.dat.node)
            <= self.parameters['truck_capacity'], name=f"capacity"
        )

        # 12) If route serves customers/orders i then j, the latter must occur
        # after the travel time from the former
        for i in self.dat.node:
            for j in self.dat.order:
                if i == j:
                    continue
                sub_mdl.addConstr(
                    s[i] + self.dat.arc[i, j]['travel_time'] - self.M * (1 - x[i, j])
                    <= s[j], f'travel_time_{i}_{j}'
                )

        return sub_mdl, x, s

    def solve(self) -> Union[None, dict[str, dict[str, Any]]]:
        """ Find a good solution to VRP using column generation and set covering.
        Uses most of the allotted solve time to iterate between solving the master
        and pricing problem to generate routes. Uses the remaining time to solve
        the master problem with binary variables to generate a collection of
        demand covering routes.

        :return: None
        """
        finding_better_routes = True
        remaining_solve_time = self.parameters['max_solve_time'] - self.init_time
        # set covering is not the hardest mip to solve, so give most of the time to column generation
        col_gen_end = time.time() + self.parameters["column_generation_solve_ratio"] * \
            remaining_solve_time

        # iterate between solving master and subproblem to generate routes
        # until we don't find improving routes or we run out of time
        while finding_better_routes and time.time() < col_gen_end:
            prev_obj = float('inf') if self.mdl.status == gu.GRB.LOADED else self.mdl.objVal
            self.mdl.optimize()
            # move on if we aren't making reasonable progress
            if self.mdl.objVal > (1 - self.parameters['min_column_generation_progress']) * prev_obj:
                break
            # reduced cost of a column = (column objective coefficient) - (row duals)^T * column coefs
            self.sub_mdl.setObjective(
                gu.quicksum(f['cost'] * self.x[i, j] for (i, j), f in self.dat.arc.items()) -
                gu.quicksum(self.c[j].pi * gu.quicksum(self.x[i, j] for i in self.dat.node if i != j)
                            for j in self.dat.order)
            )
            self.sub_mdl.optimize()
            # if negative objective, we have at least one column with a reduced cost
            if self.sub_mdl.objVal < 0:
                self._add_best_routes()
            else:
                finding_better_routes = False

        # update all route variables to binary and resolve to find a good set of covering routes
        for var in self.z.values():
            var.vtype = gu.GRB.BINARY
        self.mdl.setParam("TimeLimit", .1*remaining_solve_time)
        self.mdl.optimize()

        return self._save_solution()

    def _add_best_routes(self) -> None:
        """ Add to the master problem the best routes found by the pricing problem
        and save their stop orders and arrival times in the route dictionary

        :return: None
        """

        route_idx = len(self.z)
        solution_number = 0
        self.sub_mdl.setParam("SolutionNumber", solution_number)

        # iterate through columns with reduced costs found by gurobi
        while solution_number < self.sub_mdl.SolCount and self.sub_mdl.PoolObjVal < 0:
            route_cost = sum(f['cost'] * self.x[i, j].xn for (i, j), f in self.dat.arc.items())
            route = self._recover_route()
            # for each customer visited, get its corresponding constraint from the
            # master (set covering) problem
            constrs = [self.c[f['node_idx']] for f in route.values() if
                       f['node_idx'] != self.depot_idx]

            # add the route as a column in the master problem
            self.z[route_idx] = self.mdl.addVar(name=f'z_{route_idx}', obj=route_cost,
                                                column=gu.Column([1]*len(constrs), constrs))
            # record the order of its stops, so we can report them later if chosen
            self.route[route_idx] = route

            route_idx += 1
            solution_number += 1
            # queue the next solution from gurobi
            self.sub_mdl.setParam("SolutionNumber", solution_number)

    def _recover_route(self):
        """ Unpack a route that the pricing problem generated

        :return: route, a dictionary that maps stop order to customer locations
        and arrival times
        """
        route = {0: {'node_idx': self.depot_idx, 'arrival': 0}}
        stop = 1
        node_idx = self._next_stop(self.depot_idx)
        while node_idx != self.depot_idx:
            route[stop] = {'node_idx': node_idx, 'arrival': self.s[node_idx].xn}
            stop += 1
            node_idx = self._next_stop(node_idx)
        route[stop] = {'node_idx': self.depot_idx, 'arrival': 24}
        return route

    def _next_stop(self, current_node_idx) -> int:
        """ Determine the index of the location visited directly after visiting
        location <current_node_idx>, as determined by this solution of the pricing
        problem

        :param current_node_idx: index of the current location in this route
        :return: the index of the next location traveled to in this route
        """
        next_stops = [j for j in self.dat.node if current_node_idx != j and
                      self.x[current_node_idx, j].xn > .9]
        assert len(next_stops) == 1, 'the model constrains this list to have one element'
        return next_stops.pop()

    def _save_solution(self) -> Union[None, dict[str, dict[str, Any]]]:
        """ Save the heuristic solution generated for the exact VRP. Record
        summary statistics and each route's details.

        :return: Optionally, a dictionary of the solution data
        """
        sln = solution_schema.TicDat()
        selected_routes = [k for k, var in self.z.items() if var.x > .9]

        # record summary stats
        sln.summary['cost'] = self.mdl.objVal
        sln.summary['routes'] = len(selected_routes)

        # record the route that each used truck takes
        for k in selected_routes:
            for stop, f in self.route[k].items():
                sln.route[k, stop] = f

        # save the solution
        if self.solution_pth:
            solution_schema.csv.write_directory(sln, self.solution_pth, allow_overwrite=True)
        else:
            return make_json_dict(solution_schema, sln, verbose=True)


if __name__ == '__main__':
    # run the column generation approach
    heuristic_vrp = HeuristicVRP(input_dict=toy_input, solution_dict=True)
    heuristic_sln = heuristic_vrp.solve()
    print('Heuristic Solution:')
    print(heuristic_sln)
