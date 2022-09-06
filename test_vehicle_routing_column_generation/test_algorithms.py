import gurobipy as gu
from math import isclose
import os
import shutil
import unittest
from unittest.mock import patch

from vehicle_routing_column_generation.algorithms import ExactVRP, HeuristicVRP, VRP
from vehicle_routing_column_generation.schemas import input_schema, solution_schema, toy_input


class TestBase(unittest.TestCase):
    def setUp(self) -> None:
        self.input_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                      'test_inputs')
        self.toy_input_pth = os.path.join(self.input_dir, 'toy_input')
        self.small_input_pth = os.path.join(self.input_dir, 'small_input')
        self.medium_input_pth = os.path.join(self.input_dir, 'medium_input')
        self.sln_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                    'test_solutions')
        self.toy_sln_pth = os.path.join(self.sln_dir, 'toy_solution')
        self.small_sln_pth = os.path.join(self.sln_dir, 'small_solution')
        self.medium_sln_pth = os.path.join(self.sln_dir, 'medium_solution')

        shutil.rmtree(self.sln_dir, ignore_errors=True)
        os.mkdir(self.sln_dir)

    def tearDown(self) -> None:
        shutil.rmtree(self.sln_dir, ignore_errors=True)


class TestVRP(TestBase):

    def test_init(self):
        vrp = VRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        self.assertTrue(input_schema.good_tic_dat_object(vrp.dat))
        self.assertTrue(vrp.parameters['truck_capacity'] == 40000)
        self.assertTrue(vrp.parameters['fleet_size'] == 2)
        self.assertTrue(vrp.parameters['max_solve_time'] == 60)
        self.assertTrue(vrp.depot_idx == 0)
        self.assertTrue(vrp.fleet == [0, 1])
        self.assertTrue(vrp.M)
        self.assertTrue(vrp.solution_pth == self.toy_sln_pth)

    def test_init_fails_asserts(self):
        bad_sln_pth = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                   'test_solootions', 'toy_input')
        self.assertRaisesRegex(AssertionError, 'the parent directory of sln_pth must exist',
                               VRP, input_pth=self.toy_input_pth, solution_pth=bad_sln_pth)
        self.assertRaisesRegex(AssertionError, 'must specify where to save solution',
                               VRP, input_pth=self.toy_input_pth)

    def test_data_checks(self):
        dat = VRP._data_checks(self.toy_input_pth)
        self.assertTrue(isinstance(dat, input_schema.TicDat))

        dat = VRP._data_checks(input_dict=toy_input)
        self.assertTrue(isinstance(dat, input_schema.TicDat))

    def test_data_checks_fails_asserts(self):
        bad_input_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                     'test_inputs', 'toi_input')
        self.assertRaisesRegex(AssertionError, 'input_pth must be a valid directory',
                               VRP._data_checks, input_pth=bad_input_dir)
        msg = 'Primary Key: 1, Foreign Key: 1, Type Constraint: 1, Check Constraint: 1'
        self.assertRaisesRegex(AssertionError, msg, VRP._data_checks,
                               input_pth=os.path.join(self.input_dir, 'dirty_input_1'))
        self.assertRaisesRegex(AssertionError, 'There can only be one depot amongst the nodes',
                               VRP._data_checks, input_pth=os.path.join(self.input_dir, 'dirty_input_2'))
        self.assertRaisesRegex(AssertionError, "No order can weigh more than truck capacity",
                               VRP._data_checks, input_pth=os.path.join(self.input_dir, 'dirty_input_3'))
        self.assertRaisesRegex(AssertionError, "There should be exactly one order for each customer",
                               VRP._data_checks, input_pth=os.path.join(self.input_dir, 'dirty_input_4'))


class TestExactVRP(TestBase):

    def test_init(self):

        # check function calls
        with patch.object(VRP, '__init__') as init, \
                patch.object(ExactVRP, '_create_exact_vrp') as create:
            mdl = gu.Model()
            create.return_value = (mdl, mdl.addVar(), mdl.addVar())
            vrp = ExactVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
            self.assertTrue(init.called)
            self.assertTrue(create.called)

        # check attributes
        vrp = ExactVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        self.assertTrue(isinstance(vrp.mdl, gu.Model))
        self.assertTrue(isinstance(vrp.x, dict))
        self.assertTrue(isinstance(vrp.s, dict))
        self.assertTrue(vrp.init_time > 0)
        for (i, j, k), var in vrp.x.items():
            self.assertTrue(i in [0, 1, 2])
            self.assertTrue(j in [0, 1, 2])
            self.assertTrue(k in [0, 1])
            self.assertTrue(isinstance(var, gu.Var))
        for i, var in vrp.s.items():
            self.assertTrue(i in [0, 1, 2])
            self.assertTrue(isinstance(var, gu.Var))

    def test_create_exact_vrp(self):
        vrp = ExactVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.optimize()
        self.assertTrue(isclose(vrp.mdl.objVal, 738.569, abs_tol=.1))

    def test_solve(self):
        vrp = ExactVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        with patch.object(vrp, '_save_solution') as ss:
            ss.return_value = 'dat'
            rtn = vrp.solve()
            self.assertTrue(ss.called)
            self.assertTrue(rtn == 'dat')
            self.assertTrue(isclose(vrp.mdl.objVal, 738.569, abs_tol=.1))

    def test_small(self):
        vrp = ExactVRP(input_pth=self.small_input_pth, solution_pth=self.small_sln_pth)
        vrp.solve()
        self.assertTrue(isclose(vrp.mdl.objVal, 2891, rel_tol=.001))

    def test_medium(self):
        vrp = ExactVRP(input_pth=self.medium_input_pth, solution_pth=self.medium_sln_pth)
        vrp.solve()
        self.assertFalse(vrp.mdl.objVal < float('inf'))

    def test_save_solution(self):
        test_args = [
            {'input_pth': self.toy_input_pth, 'solution_pth': self.toy_sln_pth},
            {'input_dict': toy_input, 'solution_dict': True}
        ]

        for test_arg in test_args:
            vrp = ExactVRP(**test_arg)
            vrp.mdl.addConstr(vrp.x[0, 2, 0] == 1)  # fix chosen route to be 0 - 2 - 1 - 0
            dat_dict = vrp.solve()
            if vrp.solution_pth:
                dat = solution_schema.csv.create_tic_dat(self.toy_sln_pth)
            else:
                dat = solution_schema.TicDat(**dat_dict)
            self.assertTrue(isclose(dat.summary['cost']['value'], 738.569, abs_tol=.1))
            self.assertTrue(dat.summary['routes']['value'] == 1)
            self.assertTrue(len(dat.route) == 4)
            self.assertTrue(dat.route[0, 0]['node_idx'] == 0)
            self.assertTrue(dat.route[0, 1]['node_idx'] == 2)
            self.assertTrue(dat.route[0, 2]['node_idx'] == 1)
            self.assertTrue(dat.route[0, 3]['node_idx'] == 0)

    def test_recover_route(self):
        vrp = ExactVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.addConstr(vrp.x[0, 2, 0] == 1)  # fix chosen route to be 0 - 2 - 1 - 0
        vrp.mdl.optimize()
        route = vrp._recover_route(0)
        stop_order = [f['node_idx'] for f in route.values()]
        self.assertTrue(stop_order == [0, 2, 1, 0])

    def test_next_stop(self):
        vrp = ExactVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.addConstr(vrp.x[0, 2, 0] == 1)  # fix chosen route to be 0 - 2 - 1 - 0
        vrp.mdl.optimize()
        self.assertTrue(vrp._next_stop(0, 0) == 2)
        self.assertTrue(vrp._next_stop(2, 0) == 1)
        self.assertTrue(vrp._next_stop(1, 0) == 0)

    def test_fixing_bad_data(self):
        vrp = ExactVRP(input_pth=os.path.join(self.input_dir, 'dirty_input_to_clean'),
                       solution_pth=self.small_sln_pth)
        vrp.solve()


class TestHeuristicVRP(TestBase):

    def test_init(self):
        # check function calls
        with patch.object(VRP, '__init__') as init, \
                patch.object(HeuristicVRP, '_create_master_problem') as cmp, \
                patch.object(HeuristicVRP, '_create_subproblem') as cs:
            mdl = gu.Model()
            cmp.return_value = (mdl, mdl.addVar(), mdl.addVar(), dict())
            cs.return_value = (mdl, mdl.addVar(), mdl.addVar())
            vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
            self.assertTrue(init.called)
            self.assertTrue(cmp.called)
            self.assertTrue(cs.called)

        # check attributes
        vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        self.assertTrue(isinstance(vrp.mdl, gu.Model))
        self.assertTrue(isinstance(vrp.sub_mdl, gu.Model))
        self.assertTrue(isinstance(vrp.c, dict))
        self.assertTrue(isinstance(vrp.z, dict))
        self.assertTrue(isinstance(vrp.x, dict))
        self.assertTrue(isinstance(vrp.s, dict))
        self.assertTrue(vrp.init_time > 0)
        for k, var in vrp.z.items():
            self.assertTrue(k in [0, 1])
            self.assertTrue(isinstance(var, gu.Var))
        for j, constr in vrp.c.items():
            self.assertTrue(j in [1, 2])
            self.assertTrue(isinstance(constr, gu.Constr))
        for (i, j), var in vrp.x.items():
            self.assertTrue(i in [0, 1, 2])
            self.assertTrue(j in [0, 1, 2])
            self.assertTrue(isinstance(var, gu.Var))
        for i, var in vrp.s.items():
            self.assertTrue(i in [0, 1, 2])
            self.assertTrue(isinstance(var, gu.Var))
        for route_idx, route in vrp.route.items():
            self.assertTrue(route_idx in [0, 1])
            for stop, f in route.items():
                self.assertTrue(stop in [0, 1, 2])
                self.assertTrue(f['node_idx'] in [0, 1, 2])
                self.assertTrue(0 <= f['arrival'] <= 24)

    def test_create_master_problem(self):
        vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.optimize()
        self.assertTrue(isclose(vrp.mdl.objVal, 1391.833, abs_tol=.1))

    def test_create_subproblem(self):
        vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.optimize()
        vrp.sub_mdl.setObjective(
            gu.quicksum(f['cost'] * vrp.x[i, j] for (i, j), f in vrp.dat.arc.items()) -
            gu.quicksum(vrp.c[j].pi * gu.quicksum(vrp.x[i, j] for i in vrp.dat.node if i != j)
                        for j in vrp.dat.order)
        )
        vrp.sub_mdl.optimize()
        # corresponds to route with all stops together
        self.assertTrue(isclose(vrp.sub_mdl.objVal, -653.264, abs_tol=.1))

    def test_solve(self):
        vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        with patch.object(vrp, '_save_solution') as ss:
            ss.return_value = 'dat'
            rtn = vrp.solve()
            self.assertTrue(ss.called)
            self.assertTrue(rtn == 'dat')
            self.assertTrue(isclose(vrp.mdl.objVal, 738.569, abs_tol=.1))

    def test_small(self):
        vrp = HeuristicVRP(input_pth=self.small_input_pth, solution_pth=self.small_sln_pth)
        vrp.solve()
        self.assertTrue(isclose(vrp.mdl.objVal, 2891, rel_tol=.01))

    def test_medium(self):
        vrp = HeuristicVRP(input_pth=self.medium_input_pth, solution_pth=self.medium_sln_pth)
        vrp.solve()
        self.assertTrue(vrp.mdl.objVal < float('inf'))

    def test_add_best_routes(self):
        vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.optimize()
        old_objective = vrp.mdl.objVal
        vrp.sub_mdl.setObjective(
            gu.quicksum(f['cost'] * vrp.x[i, j] for (i, j), f in vrp.dat.arc.items()) -
            gu.quicksum(vrp.c[j].pi * gu.quicksum(vrp.x[i, j] for i in vrp.dat.node if i != j)
                        for j in vrp.dat.order)
        )
        vrp.sub_mdl.optimize()
        vrp._add_best_routes()
        self.assertTrue(len(vrp.route) > 2, 'we should have added at least 1 route')
        vrp.mdl.optimize()
        self.assertTrue(vrp.mdl.objVal < old_objective)

    def test_recover_route(self):
        vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.optimize()
        vrp.sub_mdl.setObjective(
            gu.quicksum(f['cost'] * vrp.x[i, j] for (i, j), f in vrp.dat.arc.items()) -
            gu.quicksum(vrp.c[j].pi * gu.quicksum(vrp.x[i, j] for i in vrp.dat.node if i != j)
                        for j in vrp.dat.order)
        )
        vrp.sub_mdl.addConstr(vrp.x[0, 1] == 1)  # fix chosen route to be 0 - 1 - 2 - 0
        vrp.sub_mdl.optimize()
        vrp.sub_mdl.setParam("SolutionNumber", 0)
        route = vrp._recover_route()
        stop_order = [f['node_idx'] for f in route.values()]
        self.assertTrue(stop_order == [0, 1, 2, 0])

    def test_next_stop(self):
        vrp = HeuristicVRP(input_pth=self.toy_input_pth, solution_pth=self.toy_sln_pth)
        vrp.mdl.optimize()
        vrp.sub_mdl.setObjective(
            gu.quicksum(f['cost'] * vrp.x[i, j] for (i, j), f in vrp.dat.arc.items()) -
            gu.quicksum(vrp.c[j].pi * gu.quicksum(vrp.x[i, j] for i in vrp.dat.node if i != j)
                        for j in vrp.dat.order)
        )
        vrp.sub_mdl.addConstr(vrp.x[0, 1] == 1)  # fix chosen route to be 0 - 1 - 2 - 0
        vrp.sub_mdl.optimize()
        vrp.sub_mdl.setParam("SolutionNumber", 0)
        self.assertTrue(vrp._next_stop(0) == 1)
        self.assertTrue(vrp._next_stop(1) == 2)
        self.assertTrue(vrp._next_stop(2) == 0)

    def test_save_solution(self):

        test_args = [
            {'input_pth': self.toy_input_pth, 'solution_pth': self.toy_sln_pth},
            {'input_dict': toy_input, 'solution_dict': True}
        ]

        for test_arg in test_args:
            vrp = HeuristicVRP(**test_arg)
            vrp.mdl.optimize()
            dat_dict = vrp._save_solution()
            if vrp.solution_pth:
                dat = solution_schema.csv.create_tic_dat(self.toy_sln_pth)
            else:
                dat = solution_schema.TicDat(**dat_dict)
            self.assertTrue(isclose(vrp.mdl.objVal, 1391.833, abs_tol=.1))
            self.assertTrue(dat.summary['routes']['value'] == 2)
            self.assertTrue(len(dat.route) == 6)
            self.assertTrue(dat.route[0, 0]['node_idx'] == 0)
            self.assertTrue(dat.route[0, 1]['node_idx'] == 1)
            self.assertTrue(dat.route[0, 2]['node_idx'] == 0)
            self.assertTrue(dat.route[1, 0]['node_idx'] == 0)
            self.assertTrue(dat.route[1, 1]['node_idx'] == 2)
            self.assertTrue(dat.route[1, 2]['node_idx'] == 0)


if __name__ == '__main__':
    unittest.main()
