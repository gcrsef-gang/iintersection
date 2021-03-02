"""Tests script for iintersection.py
"""

import argparse
import os

import iintersection as ii


SCENARIO_FILES = f"{os.path.abspath(os.path.dirname(__file__))}/scenarios"


def test_generate_initial_population():
    """Tests `iintersection.generate_initial_population`.

    Uses the following intersection scenarios:
     - Two-way intersection
     - NY-7/787
     - Troy-Schenectady/Northway

    Creates a directory 'tmp' and saves generated intersections there as XML files.
    """
    try:
        os.mkdir("tmp")
    except FileExistsError:
        pass

    two_way_intersection_scenario = ii.IntersectionScenario.fromXML(f"{SCENARIO_FILES}/two-way-intersection.xml")
    population = ii.generate_inital_population(two_way_intersection_scenario)
    i = 0
    for i in range(ii.GRID_SIDELEN ** 2):
        intersection = population[i // ii.GRID_SIDELEN][i % ii.GRID_SIDELEN]
        with open(f"tmp/twi-{i}.nod.xml", "w+") as f:
            f.write(intersection.getNodeXML())
        with open(f"tmp/twi-{i}.edg.xml", "w+") as f:
            f.write(intersection.getEdgeXML())


def test_crossover():
    pass


def test_mutate():
    pass


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("function", choices=["generate_initial_population", "crossover", "mutate"])
    args = parser.parse_args()
    if args.function == "generate_initial_population":
        test_generate_initial_population()
    elif args.function == "crossover":
        test_crossover()
    elif args.function == "mutate":
        test_mutate()