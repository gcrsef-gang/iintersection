"""
A cellular genetic algorithm implementation for generating optimal traffic intersections for a
given traffic scenario.
"""

import argparse
import math
import os
import subprocess
import sys

import numpy as np
import scipy.optimize
import traci

from libiintersection import (
    PY_METRICS as METRICS, PY_BACKENDS as BACKENDS, PY_JUNCTIONTYPES as JUNCTIONTYPES,
    PY_VEHICLETYPES as VEHICLETYPES,
    PyBezierCurve as BezierCurve, PyIntersection as Intersection,
    PyIntersectionRoute as IntersectionRoute, PyIntersectionEdge as IntersectionEdge,
    PyIntersectionScenario as IntersectionScenario,
    PyIntersectionNodePointer as IntersectionNodePointer
)


# Default values; can be changed by command-line args.
BACKEND = BACKENDS["sumo"]
MAX_EVALUATIONS = 25000
POPULATION_SIZE = 400
GRID_SIDELEN = int(math.sqrt(POPULATION_SIZE))


SIMULATION_TIME = 604800
# Initial population paramters.
NUM_NODES_MEAN = 15
NUM_NODES_STDEV = 7
COORD_STDEV_FACTOR = 0.25
END_ROUTE_PROB = 0.5
MAX_LANES = 5
MAX_SPEED_LIMIT = 35  # m/s
MAX_PRIORITY = 10
LANE_WIDTH = 37

# Clearance Height + the thickness of the road (decimeters)
CLEARANCE_HEIGHT = 60
POSITION_MUTATION_FACTOR = 0.05
MUTATION_PROB = 0.1

# Changed by the bounding box of the inital input scenario
POSITION_MUTATION_CUBE_LENGTH = 0

# Parent selection parameters.
NEIGHBORHOOD_TYPE = "S_3"

# Crossover parameters.
EDGE_REPLACEMENT_PROB = 0.1

# General parameters.
PARETO_FRONT_SIZE = 20

# Random number generator.
rng = np.random.default_rng(42069)


def _get_squared_distance(p1, p2):
    """Returns the squared Euclidean distance between two points.

    Parameters
    ----------
    p1: list or tuple of int or float
        A point.
    p2: list or tuple of int or float
        A point with the same number of dimensions as `p1`.

    Returns
    -------
    int or float
        The Euclidean distance between the points.
    """
    squared_distance = 0
    for coord1, coord2 in zip(p1, p2):
        squared_distance += (coord1 - coord2) ** 2
    return squared_distance


def _get_route_from_scenario_edge(intersection, scenario_edge):
    """Returns a route from an intersection that corresponds to an edge in a scenario.

    Parameters
    ----------
    intersection: Intersection
        An intersection.
    scenario_edge: ScenarioEdge
        A scenario edge that corresponds to one of the routes in Intersection.

    Returns
    -------
    IntersectionRoute
        The route from `intersection` that corresponds to `scenario_edge`.
    """
    for route in intersection.getRoutes():
        route_nodes = route.getNodeList()
        same_start_nodes = route_nodes[0].getID() == scenario_edge.getStartNode().getID()

        # Second equality test not run if same_start_nodes is false.
        if same_start_nodes and route_nodes[-1].getID() == scenario_edge.getEndNode().getID():
            return route

        same_start_loc = route_nodes[0].getLoc() == scenario_edge.getStartNode().getLoc()
        if route_nodes[-1].getLoc() == scenario_edge.getEndNode().getLoc():
            return route
        


def _get_dominant_solution(intersection1, intersection2):
    """Returns the solution that is Pareto dominant over the other.

    If neither dominate the other, a random solution is returned.
    Assumes both solutions have had their fitnesses evaluated.

    Parameters
    ----------
    s1: Intersection
        An intersection.
    s2: Intersection
        An intersection.

    Returns
    -------
    Intersection
        The intersection that is either dominant over the other or was randomly chosen.
    bool
        Whether the chosen intersection is actually dominant over the other or if it was randomly
        chosen.
    """
    dominant = True
    intersection1safety = intersection1.getMetric(METRICS["safety"])
    intersection1efficiency = intersection1.getMetric(METRICS["efficiency"])
    intersection1emissions = intersection1.getMetric(METRICS["emissions"])
    intersection2safety = intersection2.getMetric(METRICS["safety"])
    intersection2efficiency = intersection2.getMetric(METRICS["efficiency"])
    intersection2emissions = intersection2.getMetric(METRICS["emissions"])
    counter = []
    if intersection1safety < intersection2safety:
        counter.append(1)
    elif intersection1safety > intersection2safety:
        counter.append(2)
    else:
        counter.append(0)

    if intersection1efficiency < intersection2efficiency:
        counter.append(1)
    elif intersection1efficiency > intersection2efficiency:
        counter.append(2)
    else:
        counter.append(0)

    if intersection1emissions < intersection2emissions:
        counter.append(1)
    elif intersection1emissions > intersection2emissions:
        counter.append(2)
    else:
        counter.append(0)

    if 1 not in counter:
        if 2 not in counter:
            dominant = False
            randint = rng.choice([1,2])
            if randint == 1:
                selected = intersection1
            else:
                selected = intersection2
        else:
            selected = intersection2
    elif 2 not in counter:
        selected = intersection1
    else:
        dominant = False
        randint = rng.choice([1,2])
        if randint == 1:
            selected = intersection1
        else:
            selected = intersection2
    return selected, dominant


def _sample_expanded_bbox(start_coords, end_coords, num_points):
    """Chooses random points from the bounding box of two points, expanded in all directions by half
    the distance between the points.

    Parameters
    ----------
    start_coords: tuple of int
        A 3d point.
    end_coords: tuple of int
        A 3d point.
    num_points: int
        The number of points to be sampled.

    Returns
    -------
    list of tuple of int
        A list of the sampled points.
    """
    # Choose points from bounding box, expanded by half of the distance between them.
    half_distance = 0.5 * math.sqrt(_get_squared_distance(start_coords, end_coords))
    max_x = max(start_coords[0], end_coords[0]) + half_distance
    min_x = min(start_coords[0], end_coords[0]) - half_distance
    max_y = max(start_coords[1], end_coords[1]) + half_distance
    min_y = min(start_coords[1], end_coords[1]) - half_distance
    max_z = max(start_coords[2], end_coords[2]) + half_distance
    min_z = min(start_coords[2], end_coords[2]) - half_distance

    points = []
    for _ in range(num_points):
        point = []
        point.append(rng.choice(np.arange(min_x, max_x + 1)))
        point.append(rng.choice(np.arange(min_y, max_y + 1)))
        point.append(rng.choice(np.arange(min_z, max_z + 1)))
        points.append(point)
    return points


def _transform_edge(start_node, end_node, repl_edge):
    """Transforms an edge to be able to replace another edge.

    Parameters
    ----------
    start_node: IntersectionNode
        A node.
    end_node: IntersectionNode
        A node.
    repl_edge: IntersectionEdge
        The edge to be transformed to fit the start and end nodes.

    Returns
    -------
    IntersectionEdge
        `repl_edge`, but transformed to fit specified start and end nodes.
    """
    repl_start_coords = repl_edge.getStartNode().getLoc()
    rs_x, rs_y, rs_z = repl_start_coords
    repl_end_coords = repl_edge.getEndNode().getLoc()
    re_x, re_y, re_z = repl_end_coords
    edge_start_coords = start_node.getLoc()
    es_x, es_y, es_z = edge_start_coords
    edge_end_coords = end_node.getLoc()
    ee_x, ee_y, ee_z = edge_end_coords
    new_points = [list(repl_start_coords)] + [list(p) for p in repl_edge.getShape().getHandles()] + [list(repl_end_coords)]
    
    # Translate to origin.
    for point in new_points:
        for dim in range(3):
            point[dim] -= repl_start_coords[dim]

    # Rotate around origin.
    # Z-axis rotation.
    rotation_angle = (math.atan((es_y - ee_y) / (es_x - ee_x))   # Edge angle.
                    -  math.atan((rs_y - re_y) / (rs_x - re_x)))  # Replacement angle.
    for point in new_points:
        point[0] = point[0] * math.cos(rotation_angle) - point[1] * math.sin(rotation_angle)
        point[1] = point[0] * math.sin(rotation_angle) - point[1] * math.cos(rotation_angle)
    # Y-axis rotation.
    rotation_angle = (math.atan((es_z - ee_z) / (es_x - ee_x))
                    - math.atan((rs_z - re_z) / (rs_x - re_x)))
    for point in new_points:
        point[0] = point[0] * math.cos(rotation_angle) - point[2] * math.sin(rotation_angle)
        point[2] = point[0] * math.sin(rotation_angle) - point[2] * math.cos(rotation_angle)

    # Scale.
    repl_length = math.sqrt(_get_squared_distance(repl_start_coords, repl_end_coords))
    edge_length = math.sqrt(_get_squared_distance(edge_start_coords, edge_end_coords))
    scale_factor = edge_length / repl_length
    for point in new_points:
        for dim in range(3):
            point[dim] *= scale_factor

    # Translate from origin.
    for point in new_points:
        for dim in range(3):
            point[dim] += edge_start_coords[dim]

    new_points = [tuple(p) for p in new_points]

    bezier_curve = BezierCurve(start_node, end_node, new_points[1:-1])
    new_edge = IntersectionEdge(start_node, end_node, bezier_curve, repl_edge.getNumLanes(),
                                repl_edge.getSpeedLimit(), repl_edge.getPriority())
    return new_edge


def _bezier_curve(t, handles):
    if len(handles) == 2:
        x = (1-t)*handles[0][0] + t*handles[1][0]
        y = (1-t)*handles[0][1] + t*handles[1][1]
        z = (1-t)*handles[0][2] + t*handles[1][2]
    elif len(handles) == 3:
        x = math.pow((1-t), 2)*handles[0][0] + 2*(1-t)*t*handles[1][0] + math.pow(t, 2)*handles[2][0]
        y = math.pow((1-t), 2)*handles[0][1] + 2*(1-t)*t*handles[1][1] + math.pow(t, 2)*handles[2][1]
        z = math.pow((1-t), 2)*handles[0][2] + 2*(1-t)*t*handles[1][2] + math.pow(t, 2)*handles[2][2]
    else:
        x = math.pow((1-t), 3)*handles[0][0] + 3*math.pow((1-t),2)*t*handles[1][0] + 3*(1-t)*math.pow(t,2)*handles[2][0] + math.pow(t,3)*handles[3][0]
        y = math.pow((1-t), 3)*handles[0][1] + 3*math.pow((1-t),2)*t*handles[1][1] + 3*(1-t)*math.pow(t,2)*handles[2][1] + math.pow(t,3)*handles[3][1]
        z = math.pow((1-t), 3)*handles[0][2] + 3*math.pow((1-t),2)*t*handles[1][2] + 3*(1-t)*math.pow(t,2)*handles[2][2] + math.pow(t,3)*handles[3][2]
    return x, y, z


def _2d_distance_evaluation(t_values, *args):
    handles1 = args[0]
    handles2 = args[1]
    min_good_2d_distance = args[2]

    bezier1 = _bezier_curve(t_values[0], handles1)
    bezier2 = _bezier_curve(t_values[1], handles2)
    _2d_distance = _get_squared_distance(bezier1[:-1], bezier2[:-1])
    distance = _get_squared_distance(bezier1, bezier2)
    if _2d_distance < min_good_2d_distance:
        if abs(bezier1[-1]-bezier2[-1]) < CLEARANCE_HEIGHT:
            return distance-1e6
        else:
            return distance
    else:
        return distance


def _get_edges_intersection(edge1, edge2):
    edge1_bezier = edge1.getShape()
    edge1_handles = edge1_bezier.getHandles()
    edge1_handles.insert(0, edge1_bezier.getStartNode().getLoc())
    edge1_handles.insert(-1, edge1_bezier.getEndNode().getLoc())

    edge2_bezier = edge2.getShape()
    edge2_handles = edge2_bezier.getHandles()
    edge2_handles.insert(0, edge2_bezier.getStartNode().getLoc())
    edge2_handles.insert(-1, edge2_bezier.getEndNode().getLoc())
    min_good_2d_distance = LANE_WIDTH*(edge1.getNumLanes() + edge2.getNumLanes())/2
    args = (edge1_handles, edge2_handles, min_good_2d_distance)
    bounds = ((0,1),(0,1))
    solution = scipy.optimize.minimize(_2d_distance_evaluation, (0.5, 0.5), args=args, bounds=bounds)

    if round(solution.x[0]) in [0,1]:
        if round(solution.x[1]) in [0,1]:
            return False
    min_distance = _2d_distance_evaluation(solution.x, args[0], args[1], args[2])
    if min_distance > 0:
        return False
    else:
        return True


def _check_edge_intersections(intersection, edge=None):
    intersection_edges = intersection.getUniqueEdges()
    if edge is not None:
        for intersection_edge in intersection_edges:
            if edge == intersection_edge:
                continue
            if _get_edges_intersection(edge, intersection_edge):
                return True
        return False
    else:
        for edge1 in intersection_edges:
            for edge2 in intersection_edges:
                if edge1 == edge2:
                    continue
                if _get_edges_intersection(edge1, edge2):
                    return True
        return False
                

def generate_inital_population(input_scenario, output_dir=None):
    """Generates the initial grid of solutions.

    Parameters
    ----------
    input_scenario: IntersectionScenario
        An input scenario on which is based the generation of intersections.
    output_dir: NoneType or str, optional
        The directory to which files node, edge, and solution XML files will be outputted as
        intersections are generated. If set to `None`, no files are written. Default is `None`.

    Returns
    -------
    list of list of Intersection
        The grid that makes up the starting population of solutions.
    """
    input_nodes = input_scenario.getNodes()
    # Number of nodes for each intersection (excluding input nodes).
    num_nodes = rng.normal(loc=NUM_NODES_MEAN, scale=NUM_NODES_STDEV, size=POPULATION_SIZE)
    num_nodes = np.array(num_nodes, dtype=np.int32)
    num_nodes = np.clip(num_nodes, a_min=0, a_max=None)
    intersections = []  # 2D square grid.
    # Generate the bounding box of the intersection.
    input_nodes_coords = [node.getLoc() for node in input_nodes]
    input_nodes_max_x = max([loc[0] for loc in input_nodes_coords])
    input_nodes_max_y = max([loc[1] for loc in input_nodes_coords])
    input_nodes_max_z = max([loc[2] for loc in input_nodes_coords])
    input_nodes_min_x = min([loc[0] for loc in input_nodes_coords])
    input_nodes_min_y = min([loc[1] for loc in input_nodes_coords])
    input_nodes_min_z = min([loc[2] for loc in input_nodes_coords])
    POSITION_MUTATION_CUBE_LENGTH = math.sqrt((input_nodes_max_x-input_nodes_min_x)* \
            (input_nodes_max_y-input_nodes_min_y)) * POSITION_MUTATION_FACTOR
    for i in range(POPULATION_SIZE):
        while True:
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
            node_types = rng.integers(low=0, high=len(JUNCTIONTYPES), size=num_nodes[i])
            intersection_nodes = [
                IntersectionNodePointer((x, y, z), node_type) for x, y, z, node_type
                in zip(node_x_coords, node_y_coords, node_z_coords, node_types)
            ]
            # Generate a route for each edge in the input scenario.
            intersection_routes = []
            # Keys: tuples containing start and end nodes for each edge.
            # Values: the edges themselves.
            edge_nodes = {}
            for input_edge in input_scenario.getEdges():
                start_node = input_edge.getStartNode()
                end_node = input_edge.getEndNode()

                unchosen_nodes = [n for n in range(len(intersection_nodes))]
                route_nodes = [IntersectionNodePointer.fromScenarioNode(start_node)]
                route_edges = []
                while True:
                    exit_ = False

                    # 50/50 chance of connecting the previous node to the end node of the route.
                    if rng.random() < END_ROUTE_PROB:
                        route_nodes.append(IntersectionNodePointer.fromScenarioNode(end_node))
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
                        next_node_index = rng.choice(unchosen_nodes, p=probabilities)
                        route_nodes.append(intersection_nodes[next_node_index])
                        unchosen_nodes.remove(next_node_index)
                    else:
                        # All the nodes of the intersection are in this route.
                        route_nodes.append(IntersectionNodePointer.fromScenarioNode(end_node))
                        exit_ = True

                    # Generate an edge between the two most recently added nodes in the route.

                    # Create bezier curve using random points inside bounding box of start and end nodes
                    # of the edge.
                    start_end_nodes = (route_nodes[-2], route_nodes[-1])
                    if start_end_nodes in edge_nodes.keys():
                        route_edges.append(edge_nodes[start_end_nodes])
                        continue
                    num_bezier_handles = rng.choice(3)
                    start_coords = route_nodes[-2].getLoc()
                    end_coords = route_nodes[-1].getLoc()
                    if num_bezier_handles:
                        points = _sample_expanded_bbox(start_coords, end_coords, num_bezier_handles)
                    else:
                        points = []
                    bezier_curve = BezierCurve(route_nodes[-2], route_nodes[-1], points)

                    # Create edge with random priority, speed limit, and number of lanes.
                    priority = rng.choice(np.arange(1, MAX_PRIORITY + 1))
                    num_lanes = rng.choice(np.arange(1, MAX_LANES + 1))
                    speed_limit = rng.random() * MAX_SPEED_LIMIT
                    edge = IntersectionEdge(route_nodes[-2], route_nodes[-1], bezier_curve, num_lanes,
                                            speed_limit, priority)
                    route_edges.append(edge)
                    edge_nodes[(edge.getStartNode().getID(), edge.getEndNode().getID())] = edge
                    if exit_:
                        break
                intersection_routes.append(IntersectionRoute(route_nodes, route_edges))
            intersection = Intersection(intersection_routes)
            if not _check_edge_intersections(intersection):
                # Create a new row of intersections.
                if i % GRID_SIDELEN == 0:
                    intersections.append([])
                intersections[-1].append(intersection)
                sys.stdout.write(f"\r\033[92mCreated {i + 1} intersections\033[00m")
                sys.stdout.flush()
                break

        if output_dir is not None:
            with open(os.path.join(output_dir, f"{i + 1}.sol.xml"), "w+") as f:
                f.write(intersection.getSolXML())
            with open(os.path.join(output_dir, f"{i + 1}.nod.xml"), "w+") as f:
                f.write(intersection.getNodeXML())
            with open(os.path.join(output_dir, f"{i + 1}.edg.xml"), "w+") as f:
                f.write(intersection.getEdgeXML())
    

    return intersections


def get_neighborhood(position, grid):
    """Retrieve the neighborhood of intersections at the given position in the grid.

    Parameters
    ----------
    position: list or tuple of int
        A 2-d point that is the position of an individual in the grid.
    grid: list of list of Intersection
        The grid representing the population of intersections.

    Returns
    -------
    list of Intersection
        The intersections in the neighborhood of `position`.
    """
    neighborhood_intersections = []

    # Square _3, or a 3x3 square with the center being the position
    if NEIGHBORHOOD_TYPE == "S_3":
        x_positions = [position[0]+x for x in range(-1, 2)]
        y_positions = [position[1]+y for y in range(-1, 2)]
        # for position_list in [x_positions, y_positions]:
        if x_positions[0] == -1:
            x_positions[0] = GRID_SIDELEN - 1
        elif x_positions[2] == GRID_SIDELEN:
            x_positions[2] = 0
        if y_positions[1] == -1:
            y_positions[0] = GRID_SIDELEN - 1
        elif y_positions[2] == GRID_SIDELEN:
            y_positions[2] = 0
        for x_pos in x_positions:
            for y_pos in y_positions:
                neighborhood_intersections.append(grid[x_pos][y_pos])
    return neighborhood_intersections


def select_parents(neighborhood):
    """Selects the two best individuals from a neighborhood.

    Runs a binary tournament using Pareto dominance. If one competitor is not dominant over the
    other, a random choice is made.

    Parameters
    ----------
    neighborhood: list of Intersection
        A set of individuals.

    Returns
    -------
    tuple of Intersection
        The two winners of the binary tournament.
    """
    halfway = len(neighborhood) // 2
    # shuffled_neighborhood = rng.shuffle(neighborhood)
    rng.shuffle(neighborhood)
    # neighborhoodlists = [shuffled_neighborhood[:halfway], shuffled_neighborhood[halfway:]]
    neighborhoodlists = [neighborhood[:halfway], neighborhood[halfway:]]
    parents = []
    for neighborhoodlist in neighborhoodlists:
        while len(neighborhoodlist) > 1:
            # TODO: Move this code to _get_dominant_solution and replace this code with a call to
            # that function.
            intersection1, intersection2 = neighborhoodlist[0], neighborhoodlist[1]
            selected = _get_dominant_solution(intersection1, intersection2)
            if selected == intersection1:
                neighborhoodlist.pop(0)
            else:
                neighborhoodlist.pop(1)
        parents.append(neighborhoodlist[0])
    return parents[0], parents[1]


def crossover(parents, input_scenario):
    """Crosses over two solutions and returns the offspring.

    For each edge in the input scenario, a route is chosen from one of the parents. Random edges
    from this route are replaced with random edges from any route in either of the two parents.

    Parameters
    ----------
    parents: tuple of Intersection
        Two individuals to breed.
    input_scenario: IntersectionScenario
        The input to the genetic algorithm.

    Returns
    -------
    Intersection
        The result of crossing over the two parents.
    """
    all_edges = set()
    for parent in parents:
        for route in parent.getRoutes():
            for edge in route.getEdgeList():
                all_edges.add(edge)

    child_routes = []
    for scenario_edge in input_scenario.getEdges():
        # Choose a random route from either parent that corresponds to this scenario edge.
        parent_route = _get_route_from_scenario_edge(parents[rng.choice(2)], scenario_edge)

        child_route_edges = []
        for edge in parent_route.getEdgeList():
            # Replace edges in the route with other ones randomly.
            if rng.random() < EDGE_REPLACEMENT_PROB:
                repl_edge = rng.choice([e for e in all_edges if e != edge])
                child_route_edges.append(_transform_edge(edge.getStartNode(),
                                                         edge.getEndNode(), repl_edge))
            else:
                child_route_edges.append(edge)
        child_route_nodes = [child_route_edges[0].getStartNode()]
        for edge in child_route_edges:
            child_route_nodes.append(edge.getEndNode())
        child_routes.append(IntersectionRoute(child_route_nodes, child_route_edges))

    return Intersection(child_routes) 


def mutate(solution):
    """Mutates the given solution. 

    Mutates bezier handles, node (junction) types, node locations, numbers of lanes, speed limits, 
    and edge priorities.

    Parameters
    ----------
    solution: Intersection
        An intersection. Edited in-place.
    """
    routes = solution.getRoutes()
    for route in routes:
        nodes = route.getNodeList()
        edges = route.getEdgeList()
        new_nodes = nodes[:]
        new_edges = []

        # Mutating edges.
        for e, edge in enumerate(edges):

            if rng.random() < MUTATION_PROB:
                # Adding a new node to the route:
                # replacing this edge with two new edges and one new node.
                if rng.choice(2):
                    # Create new node.
                    loc = _sample_expanded_bbox(edge.getStartNode().getLoc(),
                                                edge.getEndNode().getLoc(), 1)[0]
                    junction_type = rng.choice(list(JUNCTIONTYPES.values()))
                    new_node = IntersectionNodePointer(tuple(loc), junction_type)
                else:
                    # Choose existing node.
                    new_node = rng.choice([node for route_ in routes for node in route_.getNodeList()])
                new_nodes.insert(new_nodes.index(edge.getStartNode()), new_node)
                # Create new edges.
                for _ in range(2):
                    repl_edge = rng.choice([edge for route_ in routes for edge in route_.getEdgeList()])
                    new_edges.insert(e, _transform_edge(edge.getStartNode(),
                                                        edge.getEndNode(), repl_edge))
                continue

            mutated = False
            handles = edge.getShape().getHandles()
            if rng.random() < MUTATION_PROB:
                mutated = True
                # Mutation types: 0 - remove handle, 1 - change handle, 2 - add handle.
                num_handles = len(handles)
                if num_handles == 0:
                    mutation_type = 2
                elif num_handles == 1:
                    mutation_type = rng.choice(3)
                elif num_handles == 2:
                    mutation_type = rng.choice(2)

                if mutation_type == 0:
                    handles.pop(rng.choice(len(handles)))
                elif mutation_type == 1:
                    i = rng.choice(len(handles))
                    handles[i] = [
                        round((POSITION_MUTATION_CUBE_LENGTH * (rng.random() - 0.5)) + coord)
                        for coord in handles[i]
                    ]
                elif mutation_type == 2:
                    try:
                        i = rng.choice(len(handles))
                    except ValueError:
                        i = 1
                    handles.insert(i, _sample_expanded_bbox(edge.getStartNode().getLoc(),
                                                            edge.getEndNode().getLoc(), 1)[0])
            edge.setHandles(handles)

            if rng.random() < MUTATION_PROB:
                mutated = True
                current_speed_limit = edge.getSpeedLimit()
                if current_speed_limit == 1:
                    edge.setSpeedLimit(2)
                else:
                    if rng.choice(2):
                        edge.setSpeedLimit(current_speed_limit - 1)
                    else:
                        edge.setSpeedLimit(current_speed_limit + 1)

            if rng.random() < MUTATION_PROB:
                mutated = True
                current_lane_num = edge.getNumLanes()
                if current_lane_num == 1:
                    edge.setNumLanes(2)
                else:
                    if rng.choice(2):
                        edge.setNumLanes(current_lane_num - 1)
                    else:
                        edge.setNumLanes(current_lane_num + 1)

            if rng.random() < MUTATION_PROB:
                mutated = True
                current_priority = edge.getSpeedLimit()
                if current_priority == 1:
                    edge.setPriority(2)
                else:
                    if rng.choice(2):
                        edge.setPriority(current_priority - 1)
                    else:
                        edge.setPriority(current_priority + 1)

            # Apply mutation to all instances of this edge in the intersection.
            original_edge = route.getEdgeList()[e]
            for route_ in routes:
                edge_index = -1
                edges_ = route_.getEdgeList()
                for e_, edge_ in enumerate(edges_):
                    if edge_ == original_edge:
                        edge_index = e_
                        break
                if edge_index != -1:
                    edges_[edge_index] = edge
                route_.setEdgeList(edges_)

            new_edges.append(edge)

        # Mutating nodes.
        for node in nodes[1:-1]:

            if rng.random() < MUTATION_PROB:

                # Deleting a node from the route:
                # Replacing it and the edges that go into and out of it with a single edge.

                # Remove the in/out edges for the node.
                parents = []  # As defined in directed graphs.
                children = []  # ^
                to_remove = []
                for edge in new_edges:
                    if edge.getStartNode() == node:
                        to_remove.append(edge)
                        children.append(edge.getEndNode())
                    elif edge.getEndNode() == node:
                        to_remove.append(edge)
                        parents.append(edge.getStartNode())
                # Add replacement edges for disconnected neighbors.
                to_remove_edges = {edge.getStartNode().getID() : edge.getEndNode().getID() for edge in to_remove}
                # new_edges = []
                for edge in new_edges:
                    if edge.getStartNode().getID() in list(to_remove_edges.keys()):
                        if to_remove_edges[edge.getStartNode().getID()] != edge.getEndNode().getID():
                            new_edges.append(edge)
                # new_edges = [edge for edge in new_edges if to_remove_nodes[edge.getStartNode().getID()] != edge.getEndNode().getID()]
                for child in children:
                    for parent in parents:
                        repl_edge = rng.choice([edge for route_ in routes for edge in route_.getEdgeList()])
                        new_edges.append(_transform_edge(parent, child, repl_edge))
                new_nodes.remove(node)                        

                # Remove this node from all other routes in the intersection.
                for route_ in routes:
                    if route_ == route:
                        continue
                    # Detect the node.
                    node_present = False
                    nodes_ = route_.getNodeList()
                    for node_ in nodes_:
                        if node_ == node:
                            node_present = True
                            break
                    if not node_present:
                        continue
                    # Remove the node.
                    route_.setNodeList([node_ for node_ in nodes_ if node_ != node])

                    # Remove the in/out edges for the node.
                    parents = []  # As defined in directed graphs.
                    children = []  # ^
                    to_remove = []
                    edges_ = route_.getEdgeList()
                    for edge in edges_:
                        if edge.getStartNode() == node:
                            to_remove.append(edge)
                            children.append(edge.getEndNode())
                        elif edge.getEndNode() == node:
                            to_remove.append(edge)
                            parents.append(edge.getStartNode())
                    new_edges_route_ = [edge for edge in edges_ if edge not in to_remove]
                    # Add replacement edges for disconnected neighbors.
                    for child in children:
                        for parent in parents:
                            repl_edge = rng.choice([edge for route__ in routes for edge in route__.getEdgeList()])
                            new_edges_route_.append(_transform_edge(parent, child, repl_edge))
                    route_.setEdgeList(new_edges_route_)

                continue

            if rng.random() < MUTATION_PROB:
                attribute = rng.choice(2)
                if attribute == 0:
                    new_junction_type = rng.choice(list(JUNCTIONTYPES.values()))
                    new_node = IntersectionNodePointer(node.getLoc(), new_junction_type)
                if attribute == 1:
                    current_loc = node.getLoc()
                    new_loc = [
                        round((POSITION_MUTATION_CUBE_LENGTH * (rng.random() - 0.5)) + coord)
                        for coord in current_loc
                    ]
                    new_node = IntersectionNodePointer(tuple(new_loc), node.getJunctionType())
                # Replace `node` with `new_node`
                for n, node_ in enumerate(new_nodes):
                    if node_ == node:
                        new_nodes[n] = new_node
                for route_ in routes:
                    node_index = -1
                    for n, node_ in enumerate(new_nodes):
                        if node_ == node:
                            node_index = n
                    if node_index != -1:
                        new_nodes_route_ = route_.getNodeList()
                        new_nodes_route_[node_index] = node
                        route_.setNodeList(new_nodes_route_)
        if new_edges == []:
            raise Exception("Flagged!")
        # In-place on Intersection object because getRoutes() returns route pointers.
        route.setNodeList(new_nodes)
        route.setEdgeList(new_edges)


def evaluate_fitness(solution, intersectionscenario=None):
    """Evaluates the fitness of the given solution.

    The solution's fitness metrics are now cached, so that the getMetrics() method will return
    updated values.

    Parameters
    ----------
    solution: Intersection
        An intersection.

    Returns
    -------
    tuple of float
        The safety, emmissions, and efficiency values of the intersection, in that order.
    """
    is_valid = _check_edge_intersections(solution)
    if is_valid:
        # traci is being used
        # if BACKEND == BACKENDS["traci"]:
        with open("traci/traci.nod.xml", "w+") as f:
            f.write(intersection.getNodeXML())
        with open("traci/traci.edg.xml", "w+") as f:
            f.write(intersection.getEdgeXML())
        subprocess.run(["netconvert", "-n", "traci/traci.nod.xml", "-e", "traci/traci.edg.xml", "-o", "traci/traci.net.xml"])
        with open("traci/traci.rou.xml", "w+") as f:
            f.write(intersection.getRouteXML(intersectionscenario))

        traci.start(["sumo", "-c", "traci/traci.sumocfg"])
        step = 0
        simulation_emissions = 0
        simulation_collisions = 0
        simulation_travel_times = 0
        while step < SIMULATION_TIME:
            traci.simulationStep()

            emissions_sum = 0
            vehicle_id_list = traci.vehicle.getIDList()
            for vehicle_id in vehicle_id_list:
                emissions_sum += traci.vehicle.getCO2Emission(vehicle_id)

            collisions_num = traci.simulation.getCollidingVehiclesNumber("0")

            travel_times_sum = 0
            edge_ids = traci.edge.getIDList()
            for edge_id in edge_ids:
                travel_times_sum += traci.edge.getTraveltime(edge_id)
            
            
            simulation_emissions += emissions_sum
            simulation_collisions += collisions_num
            simulation_travel_times += travel_times_sum
        return simulation_collisions, simulation_travel_times, simulation_emissions
        # Intersection.Simulate(solution, BACKEND)
        # solution.updateMetrics(BACKEND)
    else:
        solution.markInvalid()
    # return solution.getMetric(METRICS["safety"]), solution.getMetric(METRICS["efficiency"]), solution.getMetrics(METRICS["emissions"])

def update_pareto_front(pareto_front, non_dominated, dominated):
    """Updates a Pareto front.

     - Removes the dominated solution that was previously undominated if it was previously in the front.
     - Adds the new non-dominated solution to the front.
     - If no solution was removed in the first step, removes a random solution from the front.

    Parameters
    ----------
    pareto_front: list of Intersection
        A list of non-dominated solutions. Edited in-place.
    non_dominated: Intersection
        A new non-dominated solution to potentially be added to the front.
    dominated: Intersection
        A dominated solution to be removed from the front if was previously part of it.
    """
    dominated_removed = False
    if dominated in pareto_front:
        pareto_front.remove(dominated)
        dominated_removed = True
    pareto_front.append(non_dominated)
    if not dominated_removed:
        pareto_front.pop(rng.choice(len(pareto_front)))


def optimize(input_scenario):
    """Estimates the pareto front of optimal intersections for a given traffic scenario.

    Writes the intersections to a directory 'tmp'. 

    Parameters
    ----------
    input_scenario: IntersectionScenario
        A traffic scenario which can be handled by an intersection or interchange.
    """
    est_pareto_front = []
    population = generate_inital_population(input_scenario)
    # Evaluate all solutions in the grid.
    for row in population:
        for solution in row:
            evaluate_fitness(solution, input_scenario)
    num_evaluations = POPULATION_SIZE

    print("\n")

    # Mainloop:
    while num_evaluations < MAX_EVALUATIONS:
        intermediate_population = [[] for _ in range(GRID_SIDELEN)]
        for i in range(POPULATION_SIZE):
            # Produce offspring.
            pos = (i // GRID_SIDELEN, i % GRID_SIDELEN)
            neighborhood = get_neighborhood(pos, population)
            parents = select_parents(neighborhood)
            offspring = crossover(parents, input_scenario)
            mutate(offspring)

            # Choose an individual.
            evaluate_fitness(offspring, input_scenario)
            num_evaluations += 1
            current_individual = population[pos[0]][pos[1]]
            replacement_solution, dominant = _get_dominant_solution(current_individual, offspring)

            # Place the chosen individual in the population and possibly in the Pareto front.
            intermediate_population[pos[0]].append(replacement_solution)
            if replacement_solution is offspring and dominant:
                    update_pareto_front(est_pareto_front, replacement_solution, current_individual)

            sys.stdout.write(f"\r\033[92mEvaluations: {num_evaluations} \033[00m")
            sys.stdout.flush()
        population = intermediate_population

    return est_pareto_front


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("scenario")
    parser.add_argument("-b", "--backend",
                        choices=["sumo", "vissim", "cityflow", "traci"])
    parser.add_argument("-e", "--max-evaluations", dest="max_evaluations")
    parser.add_argument("-p", "--population-size", type=int, dest="population_size")
    args = parser.parse_args()

    # Generate scenario object from xml file.
    input_scenario = IntersectionScenario(args.scenario)

    # Set constants.
    if args.backend:
            BACKEND = BACKENDS[args.backend]
    if args.max_evaluations:
        MAX_EVALUATIONS = args.max_evaluations
    if args.population_size:
        if math.sqrt(args.population_size) % 1 != 0:
            raise ValueError("Population size must be a perfect square.")
        POPULATION_SIZE = args.population_size
        GRID_SIDELEN = int(math.sqrt(POPULATION_SIZE))

    # Run optimization algorithm.
    optimized_intersections = optimize(input_scenario)

    # Output optimized intersections.
    node_output_files = [f"sol_intersection_{i}.nod.xml" for i in range(len(optimized_intersections))]
    edge_output_files = [f"sol_intersection_{i}.edg.xml" for i in range(len(optimized_intersections))]
    for i, intersection in enumerate(optimized_intersections):
        with open(node_output_files[i], "w+") as f:
            f.write(node_output_files[i], intersection.getNodeXML())
        with open(edge_output_files[i], "w+") as f:
            f.write(edge_output_files[i], intersection.getEdgeXML())