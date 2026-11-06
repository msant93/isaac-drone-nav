"""Simulator backends.

Every backend implements the same abstract interface so that the autonomy
stack never imports Isaac Sim directly. Two backends are planned:

* python_sim: a pure-NumPy 6-DoF quadrotor model (tests, CI, sweeps).
* isaac_pegasus: NVIDIA Isaac Sim 5.1.0 + Pegasus Simulator v5.1.0.
"""
