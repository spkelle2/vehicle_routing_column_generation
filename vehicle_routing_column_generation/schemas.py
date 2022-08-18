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

# solution tables
solution_schema = TicDatFactory(
    summary=[['key'], ['value']],  # routes and cost
    route=[['idx', 'stop'], ['node_idx', 'arrival']]
)

solution_schema.set_data_type('route', 'idx', must_be_int=True)
solution_schema.set_data_type('route', 'stop', must_be_int=True)
