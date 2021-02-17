"""
A cellular genetic algorithm implementation for generating optimal traffic intersections for a
given traffic scenario.
"""

import argparse
import math

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
NUM_NODES_MEAN = 15
NUM_NODES_STDEV = 7
COORD_STDEV_FACTOR = 0.25
END_PROB = 0.5
MAX_LANES = 5
MAX_SPEED_LIMIT = 35  # m/s
MAX_PRIORITY = 10

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
    # Number of nodes for each intersection (excluding input nodes).
    num_nodes = rng.normal(loc=NUM_NODES_MEAN, scale=NUM_NODES_STDEV, size=POPULATION_SIZE)
    num_nodes = np.array(num_nodes, dtype=np.int32)

    intersections = []  # 2D square grid.
    row_size = math.sqrt(POPULATION_SIZE)
    for i in range(POPULATION_SIZE):

        # Generate the nodes of the intersection.
        input_nodes_coords = [node.getLoc() for node in input_nodes]
        input_nodes_max_x = max([loc[0] for loc in input_nodes_coords])
        input_nodes_max_y = max([loc[1] for loc in input_nodes_coords])
        input_nodes_max_z = max([loc[2] for loc in input_nodes_coords])
        input_nodes_min_x = min([loc[0] for loc in input_nodes_coords])
        input_nodes_min_y = min([loc[1] for loc in input_nodes_coords])
        input_nodes_min_z = min([loc[2] for loc in input_nodes_coords])
        # Choose points on a normal distribution scaled according to the locations of the nodes in
        # the input scenario.
        node_x_coords = rng.normal(loc=(input_nodes_max_x + input_nodes_min_x) / 2,
                                   scale=(input_nodes_max_x - input_nodes_min_x) * COORD_STDEV_FACTOR,
                                   size=num_nodes[i])
        node_y_coords = rng.normal(loc=(input_nodes_max_y + input_nodes_min_y) / 2,
                                   scale=(input_nodes_max_y - input_nodes_min_y) * COORD_STDEV_FACTOR,
                                   size=num_nodes[i])
        node_z_coords = rng.normal(loc=(input_nodes_max_z + input_nodes_min_z) / 2,
                                   scale=(input_nodes_max_z - input_nodes_min_z) * COORD_STDEV_FACTOR,
                                   size=num_nodes[i])
        node_x_coords = np.array(node_x_coords, dtype=np.int32)
        node_y_coords = np.array(node_y_coords, dtype=np.int32)
        node_z_coords = np.array(node_z_coords, dtype=np.int32)
        node_types = rng.integers(low=0, high=len(JUNCTIONTYPE), size=num_nodes[i])
        intersection_nodes = [
            IntersectionNode(x, y, z, node_type) for x, y, z, node_type
            in zip(node_x_coords, node_y_coords, node_z_coords, node_types)
        ]

        # Generate a route for each edge in the input scenario.
        intersection_routes = []
        for input_edge in input_scenario:
            start_node = input_edge.getStartNode()
            end_node = input_edge.getEndNode()

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
                    # Create a rectangle between the start and end points and choose random points
                    # within that rectangle.
                    m = (start_x - end_x) / (start_y - end_y)
                    theta = math.atan(m)
                    perp_m = -1 / m
                    d1 = math.sqrt(_get_squared_distance((start_x, start_y), (end_x, end_x)))
                    r1_distances = rng.integers(low=(-0.5 * d1), high=(1.5 * d1), size=n_bezier_handles)
                    r2_distances = rng.integers(low=(-0.5 * d1), high=(0.5 * d1), size=n_bezier_handles)
                    for r1, r2 in zip(r1_distances, r2_distances):
                        x = start_x + r1 * math.cos(theta) - r2 * math.sin(theta)
                        y = start_y + r1 * math.sin(theta) + r2 * math.cos(theta)
                        points.append([x, y])
                else:
                    points = []
                bezier_curve = BezierCurve(route_nodes[-2], route_nodes[-1], points)

                # Create edge with random priority, speed limit, and number of lanes.
                priority = rng.choice(arange(1, MAX_PRIORITY + 1))
                num_lanes = rng.choice(arange(1, MAX_LANES + 1))
                speed_limit = rng.random() * MAX_SPEED_LIMIT
                edge = IntersectionEdge(route_nodes[-2], route_nodes[-1], bezier_curve, num_lanes,
                                        speed_limit, priority)
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

    # Generate scenario object from xml file.
    input_scenario = IntersectionScenario.fromXMLFile(args.scenario)

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
            f.write(node_output_files[i], intersection.getNodeXML())
        with open(edge_output_files[i], 'w+') as f:
            f.write(edge_output_files[i], intersection.getNodeXML())