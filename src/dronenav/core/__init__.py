"""Core types, coordinate-frame conventions and math utilities (quaternions, SO(3)).

Conventions used across the whole project:
    * World frame: ENU (x East, y North, z Up), metres.
    * Body frame: FLU (x Forward, y Left, z Up).
    * Quaternions: scalar-last [qx, qy, qz, qw] (same as Pegasus and SciPy).
"""
