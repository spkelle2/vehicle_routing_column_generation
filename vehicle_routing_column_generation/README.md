# vehicle_routing_column_generation

## Parameter Tuning

### truck_capacity
The maximum weight that can go on one truck/be assigned to one route. Defaults
to 40,000.

### fleet_size
The number of trucks to make available when solving the `ExactVRP`. Too many trucks
will hurt solve performance, but too few will create infeasibilities. Defaults to
10.

### max_solve_time
How much time in seconds to give either algorithm for instantiation and solve.
Defaults to 60 seconds.

### exact_vrp_mip_gap
How close relatively to the optimal solution to stop solving the `ExactVRP` and
return the incumbent solution.

### solutions_per_pricing_problem
How many solutions (with negative reduced costs) to add back to the master problem
for each solve of the subproblem. Adding more slows down master problem resolve,
but adding fewer results in more pricing problem solves. Either case can result in
reduced solution quality, so user should test for optimal value. Defaults to the
same number of orders supplied in the input data set.

### pricing_problem_mip_gap
How close relatively to the optimal solution to stop solving the pricing problem and
return the incumbent solution. Defaults to .1. Higher values return more quickly
allowing for more runs of the pricing problem, while lower values allow higher
quality columns to be generated. User should test for optimal value.

### pricing_problem_time_limit
How long to solve pricing problem before terminating the solve and returning the
incumbent solution. Defaults to 1 second. Higher values allow higher quality
columns to be generated while lower values allow for more runs of the pricing problem.
User should test for optimal value.

### min_column_generation_progress
How much improvement relative to the previous iteration that the pricing problem
must find in order to be solved again. Lower values lead to longer run times but
higher quality solutions. Defaults to .001.

### column_generation_solve_ratio
What ratio of max_solve_time remaining after initialization to spend on column
generation vs. the final set covering solve to choose routes. Defaults to .9.
Changing either way can reduce solution quality, so user should test for optimal
value.
