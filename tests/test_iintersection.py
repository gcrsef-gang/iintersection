"""Tests script for iintersection.py
"""

import argparse
import os
import random
import subprocess

import iintersection as ii


SCENARIO_FILES = f"{os.path.abspath(os.path.dirname(__file__))}/scenarios"
INIT_GEN_SCENARIOS = ["ny-7-787.sce.xml", "troy-schenectady-northway.sce.xml",
                      "two-way-intersection.sce.xml"]


def test_generate_initial_population():
    """Tests `iintersection.generate_initial_population`.

    Creates a directory 'tmp/init-gen' and saves generated intersections there as XML files.
    """
    for directory in ("tmp", "tmp/init-gen"):
        try:
            os.mkdir(directory)
        except FileExistsError:
            pass

    for scenario_file in INIT_GEN_SCENARIOS:
        scenario_dir = f"tmp/init-gen/{scenario_file.split('.')[0]}"
        try:
            os.mkdir(scenario_dir)
        except FileExistsError:
            # Delete files in directory if it already exists.
            for f in os.listdir(scenario_dir):
                os.remove(f"{scenario_dir}/{f}")

        print(f"Building scenario for {scenario_file}... ")
        scenario = ii.IntersectionScenario(f"{SCENARIO_FILES}/{scenario_file}")
        print(f"Generating population for {scenario_file}... ")
        init_pop = ii.generate_inital_population(scenario)
        i = 0
        for i in range(ii.POPULATION_SIZE):
            intersection = init_pop[i // ii.GRID_SIDELEN][i % ii.GRID_SIDELEN]
            with open(f"{scenario_dir}/{i}.nod.xml", "w+") as f:
                f.write(intersection.getNodeXML())
            with open(f"{scenario_dir}/{i}.edg.xml", "w+") as f:
                f.write(intersection.getEdgeXML())
        
        # Choose a random solution to convert to net file.
        x = random.randint(0, ii.POPULATION_SIZE)
        subprocess.run(["netconvert", f"--node-files={scenario_dir}/{x}.nod.xml",
                        f"--edge-files={scenario_dir}/{x}.edg.xml", "-o",
                        f"tmp/{scenario_file.split('.')[0]}-{x}.net.xml"])


def test_crossover():
    pass


def test_mutate():
    pass


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("function", choices=["gip", "cro", "mut"])
    args = parser.parse_args()
    if args.function == "gip":
        test_generate_initial_population()
    elif args.function == "cro":
        test_crossover()
    elif args.function == "mut":
        test_mutate()