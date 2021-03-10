"""Tests script for iintersection.py
"""

import argparse
import os
import random
import subprocess
import time

import iintersection as ii


SCENARIO_FILES = f"{os.path.abspath(os.path.dirname(__file__))}/scenarios"
INIT_GEN_SCENARIOS = ["ny-7-787.sce.xml", "troy-schenectady-northway.sce.xml",
                      "two-way-intersection.sce.xml"]


def test_generate_initial_population():
    """Tests `iintersection.generate_initial_population`.

    Creates a directory 'tmp/init-gen' and saves generated intersections there as XML files.
    Tests on all scenario files in repository.
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

        print(f"\033[92mBuilding scenario for {scenario_file}... \033[00m")
        scenario = ii.IntersectionScenario(f"{SCENARIO_FILES}/{scenario_file}")

        print(f"\033[92mGenerating population for {scenario_file}... \033[00m")
        start_time = time.time()
        init_pop = ii.generate_inital_population(scenario, output_dir=scenario_dir)
        duration = time.time() - start_time
        print(f"\033[92mdone. Generated {ii.POPULATION_SIZE} intersections in {duration} seconds\033[00m")
        
        # Choose a random solution to convert to net file.
        x = random.randint(0, ii.POPULATION_SIZE)
        subprocess.run(["netconvert", f"--node-files={scenario_dir}/{x}.nod.xml",
                        f"--edge-files={scenario_dir}/{x}.edg.xml", "-o",
                        f"tmp/{scenario_file.split('.')[0]}-{x}.net.xml"])


def test_crossover(sol_files, scenario_file):
    """Tests `iintersection.crossover`

    Creates a directory `tmp/crossover` and saves crossed over intersections there as XML files.

    Parameters
    ----------
    sol_files: tuple of str
        Paths to solution XML files of each parent intersection.
    scenario_file: str
        Path to scenario XML file for intersection scenario that corresponds to both the parents.
    """
    for directory in ("tmp", "tmp/crossover"):
        try:
            os.mkdir(directory)
        except FileExistsError:
            pass

    parents = (ii.Intersection.fromSolFile(sol_files[0]), ii.Intersection.fromSolFile(sol_files[1]))
    parent_names = tuple(file_name.split('/')[-1].split('.')[0] for file_name in sol_files)
    scenario = ii.IntersectionScenario(scenario_file)

    print(f"\033[92mCreating child of {parent_names[0]} and {parent_names[1]}...\033[00m")
    start_time = time.time()
    child = ii.crossover(parents, scenario)
    duration = time.time() - start_time
    print(f"\033[92mdone. \nCreated child in {duration} seconds.\033[00m")

    scenario_name = scenario_file.split('/')[-1].split('.')[0]
    child_path = f"tmp/crossover/{scenario_name}-{parent_names[0]}x{parent_names[1]}"
    with open(f"{child_path}.nod.xml", "w+") as f:
        f.write(child.getNodeXML())
    with open(f"{child_path}.edg.xml", "w+") as f:
        f.write(child.getEdgeXML())
    with open(f"{child_path}.sol.xml", "w+") as f:
        f.write(child.getSolXML())

    subprocess.run(["netconvert", f"--node-files={child_path}.nod.xml",
                   f"--edge-files={child_path}.edg.xml", "-o", f"{child_path}.net.xml"])


def test_mutate(sol_file):
    """Tests `iintersection.mutate`

    Creates a directory `tmp/mutate` and saves mutated intersections there as XML files.

    Parameters
    ----------
    sol_file: str
        Path to solution XML file of an intersection.
    """
    for directory in ("tmp", "tmp/mutation"):
        try:
            os.mkdir(directory)
        except FileExistsError:
            pass

    intersection = ii.Intersection.fromSolFile(sol_file)
    ii.mutate(intersection)
    mutated_intersection_path = f"tmp/mutation/{sol_file.split('/')[-1].split('.')[0]}-mutated"
    with open(f"{mutated_intersection_path}.sol.xml", "w+") as f:
        f.write(intersection.getSolXML())
    with open(f"{mutated_intersection_path}.nod.xml", "w+") as f:
        f.write(intersection.getNodeXML())
    with open(f"{mutated_intersection_path}.edg.xml", "w+") as f:
        f.write(intersection.getEdgeXML())

    subprocess.run(["netconvert", f"--node-files={mutated_intersection_path}.nod.xml",
                   f"--edge-files={mutated_intersection_path}.edg.xml", "-o",
                   f"{mutated_intersection_path}.net.xml"])


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-g", "--generate-initial-population", action="store_true",
                        dest="generate_initial_population")
    parser.add_argument("-c", "--crossover", dest="crossover", type=str, nargs=3)
    parser.add_argument("-m", "--mutate", dest="mutate", type=str)
    args = parser.parse_args()
    if args.generate_initial_population:
        test_generate_initial_population()
    if args.crossover:
        test_crossover((args.crossover[0], args.crossover[1]), args.crossover[2])
    if args.mutate:
        test_mutate(args.mutate)