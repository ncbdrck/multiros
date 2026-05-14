#!/bin/python3
"""
Thin re-export of the canonical gym proxy class.

The actual implementation lives in ``uniros._proxy.GymProxy``. Before
Round 8 the same class was triplicated here, in realros/core.py, and
in uniros/core.py; every bug fix had to land three times. Now it is
defined once in UniROS and re-exported by multiros and realros.

Usage (unchanged from earlier versions):
    from multiros.core import MultirosGym as gym
    env = gym.make("env_name", args)
    env.reset()
"""

from uniros._proxy import GymProxy

# Historical class name preserved for backwards compatibility.
# ``MultirosGym`` is an alias for ``GymProxy``; ``isinstance(env,
# MultirosGym)`` continues to return True for envs created through
# any of the three packages.
MultirosGym = GymProxy

__all__ = ["MultirosGym", "GymProxy"]
