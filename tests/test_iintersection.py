"""Tests script for iintersection.py
"""

import argparse

import iintersection


def test_generate_initial_population():
    """Tests `iintersection.generate_initial_population`.

    Uses the following intersection scenarios:
     - Two-way intersection
     - NY-7/787
     - Troy-Schenectady/Northway
    """


def test_crossover():
    pass


def test_mutate():
    pass


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("function", choices=["generate_initial_population", "crossover", "mutate"])
    args = parser.parse_args()
    if args.function == "generate_initial_population":
        test_generate_initial_population()
    elif args.function == "crossover":
        test_crossover()
    elif args.function == "mutate":
        test_mutate()