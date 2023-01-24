# vehicle_routing_column_generation
A demo on how to use column generation within a heuristic for solving the Vehicle
Routing Problem with Time Windows (VRPTW). Companion repo for the medium article
["How to Implement Column Generation for Vehicle Routing"](https://medium.com/@sean-patrick-kelley/how-to-implement-column-generation-for-vehicle-routing-bdb8027c957f).

## Install/Run Instructions
This package recreates its environment from [conda](https://docs.conda.io/en/latest/miniconda.html)
and requires a [Gurobi license](https://www.gurobi.com/free-trial/) to run. After
installing both, clone this repo and do the following:
```
cd <path/to/cloned/repo>
conda env create -f environment.yml
conda activate col_gen
cd vehicle_routing_column_generation
python algorithms.py
```

To run this from your own CSV data, check out `tutorial/csv_example.py` for an
example on how to do so.

## Package Directory

### vehicle_routing_column_generation
Where the algorithms and data requirements for solving the VRPTW live. You
can also find notes on how to tune parameters here.

### test_vehicle_routing_column_generation
The test suite for vehicle_routing_column_generation can be found here.

### gists
This is the source for the gists embedded in the medium article about this repo.

### slides
This is the source and a `.pdf` rendering of a presentation I made based off the medium article.

### tutorial
This is a package for creating and running numerical experiments that compare the solution
quality of exact and heuristic solves of the VRP. I used this package to demonstrate
how column generation heuristics can find nearly optimal solutions to the VRPTW in
less time than exact solves. It also demonstrates how to run experiments in
parallel on server clusters running torque, as I used that demo to my Ph.D.
program as the excuse to get to write the rest of this project.
