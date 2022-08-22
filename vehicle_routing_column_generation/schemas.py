"""
Define the input and output data requirements as well as a static dataset that
can be loaded from memory
"""

from ticdat import TicDatFactory


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
input_schema.add_parameter("fleet_size", 10, must_be_int=True)
input_schema.add_parameter("max_solve_time", 60)
input_schema.add_parameter("exact_vrp_mip_gap", .01, max=1)
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
