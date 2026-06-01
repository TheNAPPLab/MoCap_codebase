"""
figure8_mpc_demo.py
 
Flies a single Crazyflie along a Lissajous figure-8 path using the MPC
controller from mpc_bridge.py.
 
The key difference from mpc_hover.py:
  - solve() receives a full (N+1)-step reference horizon (list of lists),
    not a single target point.  This lets the Julia solver see ahead along
    the curve and plan anticipatory velocity commands.
  - Velocity is sent directly via cf.cmdVelocityWorld() — no position
    integration, because the MPC already plans over its horizon.
  - Yaw is extracted from the mocap quaternion, not hardcoded to 0.
 
Control loop (runs at RATE Hz):
  1. Pull fresh pose from mocap (/poses topic)
  2. Build an (N+1)-step reference horizon from the analytic figure-8 curve
  3. Call mpc_bridge.solve(current_state, x_ref_horizon)  →  [vx, vy, vz, yaw_rate]
  4. Send the velocity command via cf.cmdVelocityWorld()
  5. Check tracking error and lap count; break when done
 
Usage (stack must already be running):
  python3 figure8_mpc_demo.py
 
Prerequisites:
  - ros2 launch crazyflie launch.py backend:=cpp mocap:=True gui:=False teleop:=False
  - Motive streaming NatNet before launch
  - cfB placed at its initial position with a fresh battery
"""
 
import importlib.util
import math
from pathlib import Path
 
import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
 
from crazyflie_py import Crazyswarm
from motion_capture_tracking_interfaces.msg import NamedPoseArray
 
from geometry_msgs.msg import PoseStamped

try:
    from mpc_bridge import solve
except (ImportError, ModuleNotFoundError):
    spec = importlib.util.spec_from_file_location(
        "mpc_bridge",
        Path(__file__).resolve().parent / "mpc_bridge.py",
    )
    mpc_bridge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mpc_bridge)
    solve = mpc_bridge.solve
 
# ── Must match Julia: N=30, Ts=0.05 in NonlinearMPCController ────────────────
# These are fixed by the Julia solver — do not change without updating MPCJulia.jl
MPC_N  = 30    # prediction horizon steps  (N in MPCJulia.jl)
MPC_TS = 0.05  # solver sample time in seconds  (Ts in MPCJulia.jl)
 
# ── Trajectory parameters ─────────────────────────────────────────────────────
HOVER_HEIGHT = 0.5   # m — takeoff and flight altitude
SCALE_X      = 0.35  # m — figure-8 half-extent in X  (±SCALE_X)
SCALE_Y      = 0.35  # m — figure-8 half-extent in Y  (±SCALE_Y)
PERIOD       = 8.0   # s — time for one complete figure-8 lap
NUM_LAPS     = 2     # number of laps before landing
 
# ── Control parameters ────────────────────────────────────────────────────────
RATE        = 20.0  # Hz — MPC loop rate (1/RATE should be close to MPC_TS)
TAKEOFF_DUR = 2.5   # s  — PID takeoff duration
 
# ── Safety limits ─────────────────────────────────────────────────────────────
MAX_TRACKING_ERROR_M = 0.5  # m — abort if drone deviates this far from reference
MOCAP_TIMEOUT_S      = 8.0  # s — give up waiting for mocap after this long
ARM_TIMEOUT_S        = 5.0  # s — give up waiting for arm confirmation
STABLE_TIME_S        = 5.0  # s — require stable tracking for this long before starting MPC
 
SUPERVISOR_INFO_IS_ARMED = 2
SUPERVISOR_INFO_CAN_FLY  = 8
# ─────────────────────────────────────────────────────────────────────────────
 





def wait_for_mocap(server, mocap_state: dict, timeout_s: float) -> None:
    """
    Block until at least one mocap pose has been received, or raise on timeout.

    Args:
        server: allcfs node (used for spin and clock)
        mocap_state: shared dict; populated by the mocap_callback closure
        timeout_s: maximum seconds to wait
    """
    print("Waiting for mocap lock...")
    start = server.get_clock().now()

    while rclpy.ok():
        rclpy.spin_once(server, timeout_sec=0.1)

        if mocap_state["pos"] is not None:
            print(f"Mocap lock acquired. Z = {mocap_state['pos'][2]:.3f} m")
            return

        elapsed = server.get_clock().now() - start
        if elapsed > Duration(seconds=timeout_s):
            raise RuntimeError(
                f"No mocap data received after {timeout_s} s. Check Motive."
            )


def wait_for_arm(server, cf, timeout_s: float) -> None:
    """
    Block until the drone reports IS_ARMED | CAN_FLY, or raise on timeout.

    Args:
        server: allcfs node
        cf: single Crazyflie handle
        timeout_s: maximum seconds to wait
    """
    print("Waiting for arm confirmation...")
    start = server.get_clock().now()

    while rclpy.ok():
        rclpy.spin_once(server, timeout_sec=0.1)

        supervisor = int(cf.get_status().get("supervisor", 0))
        armed_and_ready = SUPERVISOR_INFO_IS_ARMED | SUPERVISOR_INFO_CAN_FLY
        if (supervisor & armed_and_ready) == armed_and_ready:
            print("Armed and ready to fly.")
            return

        elapsed = server.get_clock().now() - start
        if elapsed > Duration(seconds=timeout_s):
            raise RuntimeError("Failed to arm drone within timeout.")

# Conversion to get Yaw from quaternion if needed in the future
def yaw_from_quat(q):
    # q = [x, y, z, w]
    x, y, z, w = q
    if w == 0.0 and x == 0.0 and y == 0.0 and z == 0.0:
        return 0.0
    return math.atan2(
        2.0 * (q[3] * q[2] + q[0] * q[1]),
        1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    )

def figure8_reference(t: float, cx: float, cy: float) -> tuple[list, list]:
    """
    Evaluate the Lissajous figure-8 reference at time t.

    The curve is:
        x(t) = cx + SCALE_X * sin(omega * t)
        y(t) = cy + SCALE_Y * sin(2 * omega * t)
        z(t) = HOVER_HEIGHT

    Returns:
        ref_pos: [x, y, z, yaw]  — reference position at time t
    """
    omega = 2.0 * math.pi / PERIOD

    # Position
    ref_pos = [cx + SCALE_X * math.sin(omega * t),
                cy + SCALE_Y * math.sin(2 * omega * t),
                HOVER_HEIGHT,
                0.0
               ]

    # acceleration (second derivative) — feedforward for cmdFullState
    ax = -SCALE_X * omega**2 * math.sin(omega * t)
    ay = -4.0 * SCALE_Y * omega**2 * math.sin(2.0 * omega * t)
    ref_acc = [ax, ay, 0.0]

    return ref_pos, ref_acc

def build_horizon(t: float, cx: float, cy: float) -> list:
    """
    Build the full (N+1)-step reference horizon expected by mpc_bridge.solve().
 
    Calls figure8_reference() at t, t+MPC_TS, t+2*MPC_TS, ..., t+MPC_N*MPC_TS.
    Sampling at MPC_TS matches Ts=0.05 inside MPCJulia.jl so the solver's
    forward-Euler prediction and the reference are on the same time grid.
 
    Returns:
        list of (MPC_N + 1) lists, each [x, y, z, yaw]
        i.e. x_ref_horizon[k] = reference state at time t + k*MPC_TS
 
        implement as a list comprehension over range(MPC_N + 1)
    """

    return [
        figure8_reference(t + k * MPC_TS, cx, cy)[0]
        for k in range(MPC_N + 1)
    ]


def run_mpc_trajectory(timeHelper, server, cf, mocap_state: dict, initial_position: list) -> None:
    """
    Main MPC trajectory loop. Runs at RATE Hz for NUM_LAPS full figure-8 laps.
 
    At each tick:
      1. rclpy.spin_once() to get the freshest mocap pose
      2. Extract current = [x, y, z, yaw] from mocap_state
         - use yaw_from_quat() if mocap_state["quat"] is available
         - fall back to [cx, cy, HOVER_HEIGHT, 0.0] on mocap dropout
      3. build_horizon(t, cx, cy)  →  x_ref_horizon  (list of N+1 lists)
      4. Compute tracking error = distance(current[:3], x_ref_horizon[0][:3])
         - abort if error > MAX_TRACKING_ERROR_M
      5. solve(current, x_ref_horizon)  →  control = [vx, vy, vz, yaw_rate]
      6. cf.cmdVelocityWorld(control[0], control[1], control[2], control[3])
      7. timeHelper.sleepForRate(RATE)
      8. Break when t > PERIOD * NUM_LAPS
 
    Args:
        timeHelper: Crazyswarm time helper
    f"  ctrl=({control[0]:.3f}, {control[1]:.3f}
        server:     allcfs node (used for spin)
        cf:         single Crazyflie handle
        mocap_state: shared dict with keys "pos" ([x,y,z]) and "quat" ([x,y,z,w])
    """
    # Wait until we have a real pose, then centre the trajectory there
    while mocap_state["pos"] is None:
        rclpy.spin_once(server, timeout_sec=0.05)
    cx, cy = mocap_state["pos"][0], mocap_state["pos"][1]
    print(f"Trajectory centred on actual position: cx={cx:.3f}, cy={cy:.3f}")

    total_duration = NUM_LAPS * PERIOD
    loop_start = timeHelper.time()

    while not timeHelper.isShutdown():

        t = timeHelper.time() - loop_start
        print(f"\n=== t={t:.2f} s / {total_duration:.2f} s ===")

        # ── Exit condition ────────────────────────────────────────────────────
        if t > total_duration:
            landing_time = timeHelper.time()
            while timeHelper.time() - landing_time < 2.0:
                x_ref_horizon = [list(initial_position) for _ in range(MPC_N + 1)]
                p = mocap_state["pos"]
                yaw = yaw_from_quat(mocap_state["quat"]) if mocap_state["quat"] is not None else 0.0
                current = [p[0], p[1], p[2], yaw]
                control = solve(current, x_ref_horizon)
                cf.cmdFullState(
                    pos=initial_position[:3],
                    vel=[control[0], control[1], control[2]],
                    acc=[0.0, 0.0, 0.0],
                    yaw=initial_position[3],
                    omega=[0.0, 0.0, control[3]]
                )
                timeHelper.sleepForRate(RATE)
            break

        # ── Step 1: spin ROS to get fresh mocap ──────────────────────────────
        rclpy.spin_once(server, timeout_sec=0.0)

        if mocap_state["pos"] is not None:
            p = mocap_state["pos"]
            yaw = yaw_from_quat(mocap_state["quat"]) if mocap_state["quat"] is not None else 0.0
            current = [p[0], p[1], p[2], yaw]
        else:
            # Fallback: use initial position at hover height
            current = [cx, cy, HOVER_HEIGHT, 0.0]

        x_ref_horizon = build_horizon(t, cx, cy)

        # ── Step 3: safety — abort if tracking error is too large ────────────
        tracking_error = math.dist(current[:3], x_ref_horizon[0][:3])
        if tracking_error > MAX_TRACKING_ERROR_M:
            print(
                f"[ABORT] Tracking error {tracking_error:.3f} m exceeds "
                f"limit {MAX_TRACKING_ERROR_M} m. Landing."
            )
            break

        ref_pos, ref_acc = figure8_reference(t, cx, cy)  # for logging and potential feedforward

        # ── Step 4: call MPC solver ───────────────────────────────────────────
        # solve() expects [x, y, z, yaw] for both arguments.
        # ref_vel is available here as feedforward if you extend mpc_bridge.solve()
        # to accept a third argument in the future.
        control = solve(current, x_ref_horizon) # returns [vx, vy, vz, yaw_rate]
        # control = [vx, vy, vz, yaw_rate]

        # ── Step 5: send full state command to the drone ───────────────────────
        
        cf.cmdFullState(
            pos=ref_pos[:3],
            vel=[control[0], control[1], control[2]],
            acc=ref_acc,
            yaw=ref_pos[3],
            omega=[0.0, 0.0, control[3]]
        )

        print(
            f"t={t:.2f}s  ref=({x_ref_horizon[0][0]:.3f}, {x_ref_horizon[0][1]:.3f}, {x_ref_horizon[0][2]:.3f})"
            f"  err={tracking_error:.3f}m"
            f"  ctrl=({control[0]:.3f}, {control[1]:.3f}, {control[2]:.3f})"
        )

        timeHelper.sleepForRate(RATE)


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    server = swarm.allcfs
    cf = server.crazyflies[0]
    cf_name = cf.prefix.lstrip("/")
 
    # ── Shared mocap state (written by callback, read by control loop) ────────
    mocap_state = {"pos": None, "quat": None}

    # ── Sim pose fallback — populates mocap_state from /cfB/pose when
    #    there is no real mocap stream (backend:=sim) ──────────────────
    

    def mocap_callback(msg):
        for named_pose in msg.poses:
            if named_pose.name == cf_name:
                p = named_pose.pose.position
                q = named_pose.pose.orientation
                mocap_state["pos"]  = [p.x, p.y, p.z] # type: ignore
                mocap_state["quat"] = [q.x, q.y, q.z, q.w] # type: ignore
                break
 
    server.create_subscription(
        NamedPoseArray,
        "poses",
        mocap_callback,
        qos_profile_sensor_data,
    )

    def sim_pose_callback(msg):
        if mocap_state["pos"] is None:
            p = msg.pose.position
            q = msg.pose.orientation
            mocap_state["pos"]  = [p.x, p.y, p.z] # type: ignore
            mocap_state["quat"] = [q.x, q.y, q.z, q.w] # type: ignore

    server.create_subscription(
        PoseStamped,
        f"/{cf_name}/pose",
        sim_pose_callback,
        10,
    )
 
    # ── Startup sequence ──────────────────────────────────────────────────────
    wait_for_mocap(server, mocap_state, MOCAP_TIMEOUT_S)
    
    print(f"Arming {cf_name}...")
    cf.arm(True)
    wait_for_arm(server, cf, ARM_TIMEOUT_S)
 
    print(f"Taking off to {HOVER_HEIGHT} m...")
    cf.takeoff(targetHeight=HOVER_HEIGHT, duration=TAKEOFF_DUR)
    

    # ── Warm Up Sequence ──────────────────────────────────────────────────────
    rclpy.spin_once(server, timeout_sec=0.0)
    p = mocap_state["pos"]
    yaw = yaw_from_quat(mocap_state["quat"]) if mocap_state["quat"] is not None else 0.0
    current = [p[0], p[1], p[2], yaw] # type: ignore
    x_ref_horizon = build_horizon(0.0, p[0], p[1]) # type: ignore
    solve(current, x_ref_horizon)

 
    timeHelper.sleep(TAKEOFF_DUR + 1.0)
    print(f"Post-takeoff mocap state: {mocap_state['pos']}")    

    initial_position = [mocap_state["pos"][0], mocap_state["pos"][1], HOVER_HEIGHT, 0.0] # type: ignore
    # ── MPC trajectory ────────────────────────────────────────────────────────
    print(f"Starting figure-8 MPC trajectory ({NUM_LAPS} lap(s))...")
    
    run_mpc_trajectory(timeHelper, server, cf, mocap_state, initial_position)
 
    # ── Landing ───────────────────────────────────────────────────────────────
    print("Landing...")
    cf.notifySetpointsStop(remainValidMillisecs=100)
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3.0)
    print("Done.")
 
 
if __name__ == "__main__":
    main()