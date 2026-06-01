"""
mpc_bridge.py

Python/Julia bridge for the MPC controller.
Exposes a single solve(current_state, x_ref_horizon) function that calls
NonlinearMPCController in Julia and returns the first control input.

Args:
    current_state:  [x, y, z, yaw]          — current drone pose from mocap
    x_ref_horizon:  list of (N+1) lists,    — look-ahead reference trajectory
                    each [x, y, z, yaw]       sampled at t, t+Ts, ..., t+N*Ts

Returns:
    [vx, vy, vz, yaw_rate]                  — first optimal control input
"""

import os
import juliacall

# ── Initialize Julia runtime and load MPC module once at import time ──────────
_jl = juliacall.newmodule("MPCBridge")

_mpc_path = os.path.join(os.path.dirname(__file__), "MPCJulia.jl")
_jl.seval(f'include("{_mpc_path}")')
_jl.seval("using .MPCJulia")

_to_float64_vector = _jl.seval("x -> Float64.(collect(x))")

# Reshape a flat Python list into a Julia (nx × ncols) Float64 matrix.
# We flatten column-major (iterate states i first, then steps k) so Julia's
# reshape(flat, nx, ncols) produces the correct (4 × N+1) layout that
# NonlinearMPCController expects.
_to_float64_matrix = _jl.seval(
    "(flat, nx, ncols) -> reshape(Float64.(collect(flat)), nx, ncols)"
)

_first_control_input = _jl.seval("u -> Vector{Float64}(u[:, 1])")
# ─────────────────────────────────────────────────────────────────────────────


def solve(current_state: list, x_ref_horizon: list) -> list:
    """
    Call the Julia MPC solver and return the first control input.

    Args:
        current_state:   [x, y, z, yaw]  current drone pose from mocap
        x_ref_horizon:   list of (N+1) lists, each [x, y, z, yaw],
                         covering the next N*Ts seconds of the reference curve.
                         Length must match N+1 = 31 (N=30 in MPCJulia.jl).

    Returns:
        [vx, vy, vz, yaw_rate]  first control input to send to drone
    """
    try:
        current_state_jl = _to_float64_vector(current_state)

        # Flatten column-major: iterate states (i) in outer loop, steps (k) inner.
        # This matches Julia's column-major memory layout so reshape(flat, 4, N+1)
        # gives x_ref[:, k] = [x, y, z, yaw] at horizon step k.
        nx     = 4
        n_cols = len(x_ref_horizon)   # N+1 = 31
        flat   = [x_ref_horizon[k][i] for i in range(nx) for k in range(n_cols)]

        x_ref_jl = _to_float64_matrix(flat, nx, n_cols)

        _, control_matrix = _jl.MPCJulia.NonlinearMPCController(
            current_state_jl, x_ref_jl
        )
        return list(_first_control_input(control_matrix))

    except Exception as e:
        print(f"[MPC WARNING] Solver failed: {e}")
        print("[MPC WARNING] Returning zero control input")
        return [0.0, 0.0, 0.0, 0.0]


if __name__ == "__main__":
    import math

    print("Initializing bridge test...")

    N   = 30
    Ts  = 0.05
    cx, cy = 0.0, 0.0
    omega  = 2.0 * math.pi / 8.0   # PERIOD = 8 s

    current = [0.0, 0.0, 0.5, 0.0]

    # Build a small figure-8 horizon starting at t=0
    x_ref_horizon = [
        [
            cx + 0.35 * math.sin(omega * k * Ts),
            cy + 0.18 * math.sin(2.0 * omega * k * Ts),
            0.5,
            0.0,
        ]
        for k in range(N + 1)
    ]

    print(f"Current state:   {current}")
    print(f"Horizon length:  {len(x_ref_horizon)} steps")
    print(f"First waypoint:  {x_ref_horizon[0]}")
    print(f"Last  waypoint:  {x_ref_horizon[-1]}")
    print("Calling solver...")

    result = solve(current, x_ref_horizon)

    print(f"Control output: {result}")
    print(f"  vx       = {result[0]:.4f} m/s")
    print(f"  vy       = {result[1]:.4f} m/s")
    print(f"  vz       = {result[2]:.4f} m/s")
    print(f"  yaw_rate = {result[3]:.4f} rad/s")