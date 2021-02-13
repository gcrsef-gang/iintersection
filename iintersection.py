# from libiintersection import sumointerface
TERMINATION_CONDITION = 25000
POPULATION_SIZE = 400


def generate_inital_population():
    """
    Generates the initial grid of solutions.
    """
    pass

def select_parents(solution):
    """
    Finds the neighborhood of the given solution 
    and runs binary tournaments decided with 
    pareto dominance, and returns the two best results. 
    """
    pass

def crossover(parent1, parent2):
    """
    Crosses over two solutions and returns the offspring.
    """
    pass

def mutate(solution):
    """
    Mutates the given solution. 
    """
    pass

def evaluate(solution):
    """
    Evaluates the given solution.
    """
    pass

if __name__ == '__main__':
    pass