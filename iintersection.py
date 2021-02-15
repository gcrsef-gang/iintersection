"""
A cellular genetic algorithm implementation for generating optimal traffic intersections for a
given traffic scenario.
"""

import argparse
import math
import xml.etree.ElementTree as ET

import numpy as np

from libiintersection import (
    BACKENDS,
    VEHICLE_TYPES,
    IntersectionScenario,
    Node,
    ScenarioEdge
)


# Default values; can be changed by command-line args.
BACKEND = 0  # SUMO
MAX_EVALUATIONS = 25000
POPULATION_SIZE = 400


def generate_inital_population():
    """
    Generates the initial grid of solutions.
    """


def select_parents(solution):
    """
    Finds the neighborhood of the given solution and runs binary tournaments decided with pareto
    dominance, and returns the two best results. 
    """


def crossover(parent1, parent2):
    """
    Crosses over two solutions and returns the offspring.
    """


def mutate(solution):
    """
    Mutates the given solution. 
    """


def evaluate(solution):
    """
    Evaluates the given solution.
    """


def optimize(input_scenario):
    """
    Estimates the pareto front of optimal intersections for a given traffic scenario.
    """


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("-b", "--backend",
                        choices=["sumo", "vissim", "cityflow"])
    parser.add_argument("-e", "--max-evaluations", dest="max_evaluations")
    parser.add_argument("-p", "--population-size", type=int, dest="population_size")
    args = parser.parse_args()

    # Parse scenario file.
    tree = ET.parse(args.scenario)
    scenario = tree.getroot()
    # There should two children: "nodes" and "edges".
    nodes = None
    edges = None
    for child in scenario:
        if child.tag == "nodes":
            nodes = child
        elif child.tag == "edges":
            edges = child
    # Construct node objects.
    nodes_dict = {}  # Maps node IDs to nodes.
    for node in nodes:
        x, y, z = node.attrib["x"], node.attrib["y"], node.attrib["z"]
        nodes_dict[node.attrib["id"]] = Node(x, y, z)
    # Construct edge objects.
    edges_list = []
    for edge in edges:
        start_node = nodes_dict[edge.attrib["from"]]
        end_node = nodes_dict[edge.attrib["to"]]
        demand = {}
        for attrib in edge.attrib:
            if attrib[-7:] == "_demand":
                vehicle_type = VEHICLE_TYPES[attrib[:-7]]
                demand[vehicle_type] = edge.attrib[attrib]
        edges_list.append(ScenarioEdge(start_node, end_node, demand))

    input_scenario = IntersectionScenario(list(nodes_dict.values()), edges_list)

    # Set constants.
    if args.backend:
        BACKEND = BACKENDS[args.backend]
    if args.max_evaluations:
        MAX_EVALUATIONS = args.max_evaluations
    if args.population_size:
        if math.sqrt(args.population_size) % 1 != 0:
            raise ValueError("Population size must be a perfect square.")
        POPULATION_SIZE = args.population_size

    # Run optimization algorithm.
    optimized_intersections = optimize(input_scenario)

    # Output optimized intersections.
    node_output_files = [f"intersection_{i}.nod.xml" for i in range(len(optimized_intersections))]
    edge_output_files = [f"intersection_{i}.edg.xml" for i in range(len(optimized_intersections))]
    for i, intersection in enumerate(optimized_intersections):
        with open(node_output_files[i], 'w+') as f:
            f.write(node_output_files[i], intersection.get_node_xml())
        with open(edge_output_files[i], 'w+') as f:
            f.write(edge_output_files[i], intersection.get_edge_xml())