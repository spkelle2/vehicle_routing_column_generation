from math import asin, cos, floor, radians, sin, sqrt
import numpy as np
import os
import sys
from typing import Tuple

from vehicle_routing_column_generation.schemas import input_schema


def random_coordinates(min_lat: float = 39.5, max_lat: float = 41,
                       min_long: float = -74.5, max_long: float = -77) -> Tuple[float, float]:
    """ Generate random geographical coordinates (around SE Pennsylvania by default)

    :param min_lat: minimum latitude
    :param max_lat: maximum latitude
    :param min_long: minimum longitude
    :param max_long: maximum longitude
    :return: random geographical coordinate
    """
    lat = round(np.random.uniform(min_lat, max_lat), 2)
    long = round(np.random.uniform(min_long, max_long), 2)
    return lat, long


def haversine(lat1: float, long1: float, lat2: float, long2: float) -> float:
    """ Calculate the great circle distance between two points on the earth
    (specified in decimal degrees)

    :param lat1: latitude of first geographical coordinate
    :param long1: longitude of first geographical coordinate
    :param lat2: latitude of second geographical coordinate
    :param long2: longitude of second geographical coordinate
    :return: miles between two points
    """
    # convert decimal degrees to radians
    long1, lat1, long2, lat2 = map(radians, [long1, lat1, long2, lat2])
    # haversine formula
    dlon = long2 - long1
    dlat = lat2 - lat1
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    # Radius of earth in miles is 3958.8
    mi = 3958.8*c
    return mi


def main(pth: str, number_orders: int, *args) -> None:
    """ Create the random input data set to the given number of orders

    :param pth: where to save the output dictionary
    :param number_orders: how many number of orders and customers to create
    :param args: this should be empty, but a catchall for unneeded arguments
    :return:
    """
    assert not args, 'two arguments are expected from the command line'
    assert os.path.isdir(os.path.dirname(pth)), 'the parent directory of pth must exist'
    number_orders = int(number_orders)
    assert number_orders > 0, 'number orders is a positive integer'

    dat = {'arc': {}, 'node': {}, 'order': {}, 'parameters': {}}

    # create the depot
    dat['node'][0] = {'name': 'depot', 'type': 'depot'} | \
                     dict(zip(['lat', 'long'], random_coordinates())) | \
                     {'open': 0, 'close': 24}

    # create customers and their orders
    for idx in range(1, number_orders + 1):
        open = floor(np.random.uniform(6, 14))
        weight = floor(np.random.uniform(4000, 20000))
        dat['node'][idx] = {'name': f'customer {idx}', 'type': 'customer'} | \
                           dict(zip(['lat', 'long'], random_coordinates())) | \
                           {'open': open, 'close': open + 8}
        dat['order'][idx] = {'weight': weight}

        # find distances in between all nodes
        for other_idx in range(idx):
            # average distance between two points in USA is haversine milage * 1.18
            miles = haversine(dat['node'][other_idx]['lat'], dat['node'][other_idx]['long'],
                              dat['node'][idx]['lat'], dat['node'][idx]['long']) * 1.18
            time = miles/50  # travel time in hours
            # assume $1/mi and a $500 fixed cost for each route
            dat['arc'][other_idx, idx] = {'travel_time': time, 'cost': miles + (other_idx == 0)*500}
            dat['arc'][idx, other_idx] = {'travel_time': time, 'cost': miles}

    # set parameter values
    dat['parameters']['truck_capacity'] = {'value': 40000}
    dat['parameters']['fleet_size'] = {'value': number_orders//2 + 1}
    dat['parameters']['max_solve_time'] = {'value': 1200}

    dat = input_schema.TicDat(**dat)
    input_schema.csv.write_directory(dat, pth)
    print()


if __name__ == '__main__':
    main(*sys.argv[1:])
