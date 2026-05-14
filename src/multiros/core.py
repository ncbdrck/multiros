#!/bin/python3
"""
Re-export of the canonical multiprocessing gym proxy.

The implementation lives in :class:`uniros._proxy.GymProxy`.
``MultirosGym`` is an alias kept for backwards compatibility — both
names refer to the same class object, so ``isinstance(env,
MultirosGym)`` matches envs created through any package in the
ecosystem.

Usage::

    from multiros.core import MultirosGym as gym
    env = gym.make("env_name", args)
    env.reset()
"""

from uniros._proxy import GymProxy

MultirosGym = GymProxy

__all__ = ["MultirosGym", "GymProxy"]
