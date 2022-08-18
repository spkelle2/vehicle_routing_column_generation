import gurobipy as gu
import os
import time
from typing import Tuple, Any

from vehicle_routing_column_generation.schemas import input_schema, solution_schema


class VRP:

    def __init__(self, input_pth: str, solution_pth: str):
        """Base constructor for VRP. Checks the validity of the input data and
        solution path. Assigns common attributes.

        :param input_pth: The location of the directory storing input CSV data
        :param solution_pth: The location of the directory where solution CSV data will be stored
        """
        # check validity of given directories
        assert os.path.isdir(input_pth), 'input_pth must be a valid directory'
        assert os.path.isdir(os.path.dirname(solution_pth)), 'the parent directory of sln_pth must exist'

        # assign base class attributes
        self.solution_pth = solution_pth
        dat = self._data_checks(input_pth)
        self.dat = dat
        self.parameters = input_schema.create_full_parameters_dict(dat)
        self.depot_idx = [i for i, f in self.dat.node.items() if f['type'] == 'depot'].pop()
        self.fleet = list(range(self.parameters['fleet_size']))
        self.M = max(dat.node[i]['close'] + f['travel_time'] - dat.node[i]['open']
                     for (i, j), f in dat.arc.items()) + 1

    @staticmethod
    def _data_checks(input_pth: str) -> input_schema.TicDat:
        """ Read in the CSV's with data used for solving this VRP instance.
        Confirms the CSV's match the schema defined in schemas.input_schema.
        Check that the data in the CSV's match the remaining assumptions of our
        VRP models.

        :param input_pth: The directory containing the CSV's of input data for this VRP instance
        :return: a TicDat containing our input data
        """
        # basic ticdat checks
        pk_fails = input_schema.csv.find_duplicates(input_pth)
        dat = input_schema.csv.create_tic_dat(input_pth)
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


class ExactVRP(VRP):

    def __init__(self, **kwargs):
        """Constructor for exact VRP. Builds model in gurobi.

        :param **kwargs: keyword arguments to pass onto base constructor
        """
        init_start = time.time()
        super().__init__(**kwargs)
        self.mdl, self.x, self.s = self._create_exact_vrp()
        self.init_time = time.time() - init_start

    def _create_exact_vrp(self) -> \
            Tuple[gu.Model, dict[Tuple[int, int, int], gu.Var], dict[int, gu.Var]]:
        """ Create the gurobi model for the exact VRP. Formulation from
        https://how-to.aimms.com/Articles/332/332-Formulation-CVRP.html and
        https://how-to.aimms.com/Articles/332/332-Time-Windows.html

        :return: mdl, x, and s, which are respectively the gurobi model, a
        dictionary of variables representing arcs traveled, and a dictionary
        of variables representing arrival times at each customer
        """
        # make model
        mdl = gu.Model("exact_vrp")
        mdl.setParam("MIPGap", .01)

        # create variables
        # x_i_j_k if truck k travels from node i to node j
        x = {(i, j, k): mdl.addVar(vtype=gu.GRB.BINARY, name=f'x_{i}_{j}_{k}')
             for i in self.dat.node for j in self.dat.node for k in self.fleet if i != j}
        # s_i time when service begins at node i
        s = {i: mdl.addVar(lb=f['open'], ub=f['close'], name=f's_{i}')
                  for i, f in self.dat.node.items()}

        # set objective
        mdl.setObjective(gu.quicksum(
            gu.quicksum(f['cost'] * x[i, j, k] for (i, j), f in self.dat.arc.items())
            for k in self.fleet
        ), sense=gu.GRB.MINIMIZE)

        # set constraints
        # 1) Any node j entered by truck k must be left by truck k
        for j in self.dat.node:
            for k in self.fleet:
                mdl.addConstr(
                    gu.quicksum(x[i, j, k] for i in self.dat.node if i != j) -
                    gu.quicksum(x[j, h, k] for h in self.dat.node if j != h) == 0,
                    name=f"flow_conserve_{j}_{k}"
                )
        # 2) Every customer is visited once
        for j, f in self.dat.order.items():
            mdl.addConstr(
                gu.quicksum(gu.quicksum(x[i, j, k] for i in self.dat.node if i != j)
                            for k in self.fleet) == 1,
                name=f"demand_{j}"
            )
        # 3) Every truck k leaves the depot at most once
        for k in self.fleet:
            mdl.addConstr(
                gu.quicksum(x[self.depot_idx, j, k] for j in self.dat.order)
                <= 1, name=f"include_depot_{k}"
            )
        # 4) Every truck k stays within capacity
        for k in self.fleet:
            mdl.addConstr(
                gu.quicksum(gu.quicksum(f['weight'] * x[i, j, k] for j, f in self.dat.order.items()
                                        if i != j) for i in self.dat.node)
                <= self.parameters['truck_capacity'], name=f"capacity_{k}"
            )

        # 5) If truck k serves customers/orders i then j, the latter must occur
        # after the travel time from the former
        for k in self.fleet:
            for i in self.dat.node:
                for j in self.dat.order:
                    if i == j:
                        continue
                    mdl.addConstr(
                        s[i] + self.dat.arc[i, j]['travel_time'] - self.M * (1 - x[i, j, k]) <= s[j],
                        f'travel_time_{i}_{j}_{k}'
                    )

        return mdl, x, s

    def solve(self) -> None:
        """ Solve the exact VRP and save its solution

        :return: None
        """
        # limit solve time to whatever is left after model build
        self.mdl.setParam("TimeLimit", max(self.parameters['max_solve_time'] - self.init_time, .1))
        self.mdl.optimize()
        if self.mdl.objVal < float('inf'):
            self._save_solution()
        else:
            print('no solution found!')

    def _save_solution(self) -> None:
        """ Save the solution to the exact VRP. Record summary statistics and
        each route's details.

        :return: None
        """

        sln = solution_schema.TicDat()
        used_trucks = [k for k in self.fleet if
                       sum(self.x[self.depot_idx, j, k].x for j in self.dat.order) >= .9]

        # record summary stats
        sln.summary['cost'] = self.mdl.objVal
        sln.summary['routes'] = len(used_trucks)

        # record the route that each used truck takes
        for k in used_trucks:
            for stop, f in self._recover_route(truck_idx=k).items():
                sln.route[k, stop] = f

        # save the solution
        solution_schema.csv.write_directory(sln, self.solution_pth, allow_overwrite=True)

    def _recover_route(self, truck_idx) -> dict[int, dict[str, Any]]:
        """ Unpack the route that the exact VRP determined truck <truck_idx>
        traveled to make its deliveries

        :param truck_idx: the index of the truck which route needs recovering
        :return: route, a dictionary that maps stop order to customer locations
        and arrival times
        """
        route = {0: {'node_idx': self.depot_idx, 'arrival': 0}}
        stop = 1
        node_idx = self._next_stop(self.depot_idx, truck_idx)

        while node_idx != self.depot_idx:
            route[stop] = {'node_idx': node_idx, 'arrival': self.s[node_idx].x}

            stop += 1
            node_idx = self._next_stop(node_idx, truck_idx)

        route[stop] = {'node_idx': self.depot_idx, 'arrival': 24}
        return route

    def _next_stop(self, current_node_idx, truck_idx) -> int:
        """ Determine the index of the location truck <truck_idx> traveled after
        visiting location <current_node_idx>, as determined by the exact VRP model

        :param current_node_idx: index of location truck <truck_idx> is currently
        :param truck_idx: the index of the truck which route is being unpacked
        :return: the index of the next location truck <truck_idx> travels
        """
        next_stops = [j for j in self.dat.node if current_node_idx != j and
                      self.x[current_node_idx, j, truck_idx].x > .9]
        assert len(next_stops) == 1, 'the model constrains this list to have one element'
        return next_stops.pop()


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
        """ Create the gurobi model for the heuristic VRP master problem. Follows
        a set covering formulation to eventually select a collection of routes that
        fulfill all customer orders. Begins as an LP to yield dual values for
        pricing problem. Once all desired routes (columns) have been added, variables
        can be made binary to select the most cost efficient subset.

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
        # initialize the variables to represent the set of singleton routes
        route = {route_idx: {
            0: {'node_idx': self.depot_idx, 'arrival': 0},
            1: {'node_idx': j, 'arrival': max(self.dat.arc[self.depot_idx, j]['travel_time'],
                                              self.dat.node[j]['open'])},
            2: {'node_idx': self.depot_idx, 'arrival': 24}
        } for route_idx, j in singleton.items()}

        # create variables and set objective
        # z_i - if route i is chosen - begins relaxed for column generation
        z = {route_idx: mdl.addVar(obj=self.dat.arc[self.depot_idx, j]['cost'] +
                                   self.dat.arc[j, self.depot_idx]['cost'], name=f'z_{route_idx}')
             for route_idx, j in singleton.items()}

        # set constraints
        # each customer must be visited by a route (i.e. have delivery demand met)
        # since starting routes are singletons, each must be selected to cover
        c = {j: mdl.addConstr(z[route_idx] >= 1, name=f'c_{j}') for route_idx, j
             in singleton.items()}

        return mdl, z, c, route

    def _create_subproblem(self) -> \
            Tuple[gu.Model, dict[Tuple[int, int], gu.Var], dict[int, gu.Var]]:
        """ Create the gurobi model for the heuristic VRP pricing problem. Formulation
        from https://how-to.aimms.com/Articles/332/332-Formulation-CVRP.html and
        https://how-to.aimms.com/Articles/332/332-Time-Windows.html. Since this
        model generates a single route, indexing over all trucks and requiring
        that all customers are visited are excepted.

        :return: sub_mdl, x, and s, which are respectively the gurobi model, a
        dictionary of variables representing arcs traveled, and a dictionary
        of variables representing arrival times at each customer
        """
        # make model
        sub_mdl = gu.Model("vrp_subproblem")
        sub_mdl.setParam("PoolSolutions", len(self.dat.order))
        # force early termination so we get through as many subproblems as possible
        sub_mdl.setParam("MIPGap", .1)
        sub_mdl.setParam("TimeLimit", 1)

        # create variables
        # x_i_j if this route travels from node i to node j
        x = {(i, j): sub_mdl.addVar(vtype=gu.GRB.BINARY, name=f'x_{i}_{j}')
                  for i in self.dat.node for j in self.dat.node if i != j}
        # s_i time when service begins at node i
        s = {i: sub_mdl.addVar(lb=f['open'], ub=f['close'], name=f's_{i}')
                  for i, f in self.dat.node.items()}

        # set constraints
        # 1) Any node j entered by this route must be left
        for j in self.dat.node:
            sub_mdl.addConstr(
                gu.quicksum(x[i, j] for i in self.dat.node if i != j) -
                gu.quicksum(x[j, h] for h in self.dat.node if j != h) == 0,
                name=f"flow_conserve_{j}"
            )
        # 2) The route leaves the depot at most once
        sub_mdl.addConstr(
            gu.quicksum(x[self.depot_idx, j] for j in self.dat.order) <= 1,
            name=f"include_depot"
        )
        # 4) Route stays within capacity
        sub_mdl.addConstr(
            gu.quicksum(gu.quicksum(f['weight'] * x[i, j] for j, f in self.dat.order.items()
                                    if i != j) for i in self.dat.node)
            <= self.parameters['truck_capacity'], name=f"capacity"
        )

        # 5) If route serves customers/orders i then j, the latter must occur
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

    def solve(self) -> None:
        """ Find a good solution to VRP using column generation and set covering.
        Uses most of the allotted solve time to iterate between solving the master
        and pricing problem to generate routes. Uses the remaining time to solve
        the master problem with binary variables to generate a collection of
        demand covering routes.

        :return: None
        """
        finding_better_routes = True
        remaining_solve_time = self.parameters['max_solve_time'] - self.init_time
        col_gen_end = time.time() + .9*remaining_solve_time

        # iterate between solving master and subproblem to generate routes
        # until no improving routes left or we run out of time
        while finding_better_routes and time.time() < col_gen_end:
            prev_obj = float('inf') if self.mdl.status == gu.GRB.LOADED else self.mdl.objVal
            self.mdl.optimize()
            # move on if we aren't making reasonable progress
            if self.mdl.objVal > .999 * prev_obj:
                break
            # reduced cost of a column = (column objective coefficient) - (row duals)^T * column coefs
            self.sub_mdl.setObjective(
                gu.quicksum(f['cost'] * self.x[i, j] for (i, j), f in self.dat.arc.items()) -
                gu.quicksum(self.c[j].pi * gu.quicksum(self.x[i, j] for i in self.dat.node if i != j)
                            for j in self.dat.order)
            )
            self.sub_mdl.optimize()
            if self.sub_mdl.objVal < 0:
                self._add_best_routes()
            else:
                finding_better_routes = False

        # update all route variables to binary and resolve to find a good set of covering routes
        for var in self.z.values():
            var.vtype = gu.GRB.BINARY
        self.mdl.setParam("TimeLimit", .1*remaining_solve_time)
        self.mdl.optimize()

        self._save_solution()

    def _add_best_routes(self) -> None:
        """ Add to the master problem the best routes found by the pricing problem
        and save their stop orders and arrival times in the route dictionary

        :return: None
        """

        route_idx = len(self.z)
        solution_number = 0
        self.sub_mdl.setParam("SolutionNumber", solution_number)

        while solution_number < self.sub_mdl.SolCount and self.sub_mdl.PoolObjVal < 0:
            route_cost = sum(f['cost'] * self.x[i, j].xn for (i, j), f in self.dat.arc.items())
            route = self._recover_route()
            constrs = [self.c[f['node_idx']] for f in route.values() if
                       f['node_idx'] != self.depot_idx]

            # add the route as a column in the master problem
            self.z[route_idx] = self.mdl.addVar(name=f'z_{route_idx}', obj=route_cost,
                                                column=gu.Column([1]*len(constrs), constrs))
            # record the order of its stops, so we can report them later
            self.route[route_idx] = route

            route_idx += 1
            solution_number += 1
            self.sub_mdl.setParam("SolutionNumber", solution_number)
        # self.mdl.update()

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

    def _save_solution(self) -> None:
        """ Save the heuristic solution generated for the exact VRP. Record
        summary statistics and each route's details.

        :return: None
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
        solution_schema.csv.write_directory(sln, self.solution_pth, allow_overwrite=True)
