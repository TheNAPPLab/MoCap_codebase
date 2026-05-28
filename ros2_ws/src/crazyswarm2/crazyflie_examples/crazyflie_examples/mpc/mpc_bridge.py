"""
mpc_bridge.py

Python/Julia bridge for the MPC controller.
Exposes a single solve(current_state, target_state) function
that calls NonlinearMPCController in Julia and returns the
first control input as a Python list.
"""

import os
import juliacall

# ── Initialize Julia runtime and load MPC module once at import time ──────────
_jl = juliacall.newmodule("MPCBridge")

_mpc_path = os.path.join(os.path.dirname(__file__), "MPCJulia.jl")
_jl.seval(f'include("{_mpc_path}")')
_jl.seval("using .MPCJulia")
# ─────────────────────────────────────────────────────────────────────────────


def solve(current_state: list, target_state: list) -> list:
    """
    Call the Julia MPC solver and return the first control input.

    Args:
        current_state: [x, y, z, yaw] current drone pose from mocap
        target_state:  [x, y, z, yaw] target pose

    Returns:
        [vx, vy, vz, yaw_rate] first control input to send to drone
    """
    # TODO:
    # 1. Convert current_state and target_state to Julia Vector{Float64}
    # 2. Call _jl.MPCJulia.NonlinearMPCController with those vectors
    # 3. Extract the control matrix u from the returned tuple
    # 4. Extract the first column u[:,1] as the current control input
    # 5. Convert to Python list and return
    pass