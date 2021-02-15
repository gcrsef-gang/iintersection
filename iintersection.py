"""
A cellular genetic algorithm implementation for generating optimal traffic intersections for a
given traffic scenario.
"""

import argparse
import math
import xml.etree.ElementTree as ET

from numpy import arange
from numpy.random import default_rng

from libiintersection import (
    BACKENDS, VEHICLETYPES, JUNCTIONTYPE,
    BezierCurve, Intersection, IntersectionNode, IntersectionRoute, IntersectionEdge,
    IntersectionScenario, Node, ScenarioEdge
)


# Default values; can be changed by command-line args.
BACKEND = 0  # SUMO
MAX_EVALUATIONS = 25000
POPULATION_SIZE = 400

# Initial population paramters.
MAX_NODES = 20
MIN_COORD = -10000
MAX_COORD = 10000
END_PROB = 0.5
MAX_LANES = 5
MAX_SPEED_LIMIT = 35  # m/s

# Random number generator.
rng = default_rng(42069)


def _get_squared_distance(p1, p2):
    squared_distance = 0
    for coord1, coord2 in zip(p1, p2):
        squared_distance += (coord1 - coord2) ** 2
    return squared_distance


def generate_inital_population(input_scenario):
    """
    Generates the initial grid of solutions.
    """
    input_nodes = input_scenario.getNodes()
    n_input_nodes = len(input_nodes)
    # Number of nodes for each intersection (excluding input nodes).
    n_nodes = rng.integers(low=0, high=MAX_NODES + 1, size=POPULATION_SIZE)

    intersections = []  # 2D square grid.
    row_size = math.sqrt(POPULATION_SIZE)
    for i in range(POPULATION_SIZE):

        # Generate the nodes of the intersection.
        node_coords = rng.integers(low=MIN_COORD, high=MAX_COORD + 1, size=(n_nodes[i], 3))
        node_types = rng.integers(low=0, high=len(JUNCTIONTYPE), size=n_nodes[i])
        intersection_nodes = []
        for n in range(n_nodes[i]):
            x, y, z = list(node_coords[n])
            node_type = node_types[n]
            intersection_nodes.append(IntersectionNode(x, y, z, node_type))

        # Generate a route for each edge in the input scenario.
        intersection_routes = []
        for input_edge in input_scenario:
            start = input_edge.getStartNode()
            end = input_edge.getEndNode()
            start_node = [node for node in input_nodes if node.getID() == start][0]
            end_node = [node for node in input_nodes if node.getID() == end][0]

            unchosen_nodes = [n for n in range(len(intersection_nodes))]
            route_nodes = [IntersectionNode(start_node)]
            route_edges = []
            while True:
                exit_ = False

                # 50/50 chance of connecting the previous node to the end node of the route.
                if rng.choice(2, p=[END_PROB, 1 - END_PROB]):
                    route_nodes.append(IntersectionNode(end_node))
                    exit_ = True
                elif len(unchosen_nodes) > 0:
                    # Choose a random node with a probability proportional to its distance from the
                    # previous node in the route.
                    distances = []
                    for n in unchosen_nodes:
                        squared_distance = _get_squared_distance(route_nodes[-1].getLoc(),
                                                                 intersection_nodes[n].getLoc())
                        distances.append(squared_distance)
                    distance_sum = sum(distances)
                    probabilities = [d / distance_sum for d in distances]
                    next_node_index = rng.choice(intersection_nodes, p=probabilities)
                    route_nodes.append(intersection_nodes[next_node_index])
                    unchosen_nodes.remove(next_node_index)
                else:
                    # All the nodes of the intersection are in this route.
                    route_nodes.append(IntersectionNode(end_node))
                    exit_ = True

                # Generate an edge between the two most recently added nodes in the route.

                # Create bezier curve using random points inside bounding box of start and end nodes
                # of the edge.
                n_bezier_handles = rng.choice(3)
                start_x, start_y, start_z = route_nodes[-2].getLoc()
                end_x, end_y, end_z = route_nodes[-1].getLoc()
                if n_bezier_handles:
                    x_coords = rng.integers(low=min(start_x, end_x),
                                            high=max(start_x, end_x), size=n_bezier_handles)
                    y_coords = rng.integers(low=min(start_y, end_y),
                                            high=max(start_y, end_y), size=n_bezier_handles)
                    z_coords = rng.integers(low=min(start_z, end_z),
                                            high=max(start_z, end_z), size=n_bezier_handles)
                    points = [[x, y, z] for x, y, z in zip(x_coords, y_coords, z_coords)]
                else:
                    points = []
                bezier_curve = BezierCurve(route_nodes[-2], route_nodes[-1], points)

                # Create edge with random speed limit and number of lanes.
                num_lanes = rng.choice(arange(1, MAX_LANES + 1))
                speed_limit = rng.random() * MAX_SPEED_LIMIT
                edge = IntersectionEdge(route_nodes[-2], route_nodes[-1], bezier_curve, num_lanes,
                                        speed_limit)
                route_edges.append(edge)

                if exit_:
                    break
            
            intersection_routes.append(IntersectionRoute(route_nodes, route_edges))

        # Create a new row of intersections.
        if i % row_size == 0:
            intersections.append([])
        intersections[-1].append(Intersection(intersection_routes))

    return intersections


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
                vehicle_type = VEHICLETYPES[attrib[:-7]]
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