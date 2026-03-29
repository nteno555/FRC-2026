import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq
import matplotlib.pyplot as plt

# =============================================================================
# --- CONFIGURABLE PHYSICAL CONSTANTS ---
# Modifying these variables will physically change the flight trajectories.
# =============================================================================

# Game Piece Physics
BALL_MASS_KG      = 0.20
BALL_DIAMETER_M   = 0.15
AIR_DENSITY_KG_M3 = 1.204
DRAG_COEFFICIENT  = 0.47

# Robot & Field Geometry
SHOOTER_HEIGHT_M   = 0.56    # Height of shooter exit from floor
HUB_HEIGHT_M       = 1.44    # Height of target center from floor
FLYWHEEL_DIAMETER_M = 0.1016 # 4-inch wheels
GEARING_RATIO      = 0.5     # Flywheel motor shaft to wheel surface ratio
SLIP_AT_40_RPS     = 0.90    # Ball slip multiplier at 40 RPS
SLIP_AT_60_RPS     = 0.90    # Ball slip multiplier at 60 RPS

# =============================================================================
# --- MODE SELECTION ---
# "verify" : confirms time-of-flight for hand-tuned data points (original mode)
# "solve"  : optimizes flywheel RPS and hood angle for each target distance
# =============================================================================
MODE = "solve"

# =============================================================================
# --- VERIFY MODE: Hand-tuned data ---
# =============================================================================
TARGET_DISTANCES_M = [1.29, 2.47, 3.07, 4.04, 5.24]
HOOD_ANGLES_RAW    = [0.8,  1.7,  2.0,  2.4,  2.4]
FLYWHEEL_RPS_RAW   = [40.0, 43.0, 45.0, 50.0, 58.0]

# =============================================================================
# --- SOLVE MODE CONFIG ---
# =============================================================================

# Target distances to solve for
SOLVER_DISTANCES_M = [1.29, 2.00, 2.47, 3.07, 4.04, 4.50, 5.24]

# Hood angle search range (in raw rotation units, same as hoodTreeMap)
HOOD_SWEEP_MIN   = 0.3
HOOD_SWEEP_MAX   = 3.0
HOOD_SWEEP_STEPS = 100   # Number of candidate hood angles to evaluate

# Flywheel RPS bracket for brentq root-find (Stage 1)
RPS_SEARCH_MIN = 25.0
RPS_SEARCH_MAX = 80.0

# Finite difference step sizes for sensitivity calculation (Stage 2)
DELTA_V     = 0.5   # RPS perturbation for ∂y/∂v
DELTA_THETA = 0.02  # Hood rotation perturbation for ∂y/∂θ

# Weights for the normalized sensitivity cost function
# cost = W_V * (∂y/∂v / Δv)² + W_THETA * (∂y/∂θ / Δθ)²
W_V     = 1.0
W_THETA = 1.0

# Cone clearance constraint:
# At x = (target_distance - CONE_RIM_OFFSET_M), the ball must be above CONE_HEIGHT_M.
# This ensures the ball is descending into the cone from above, not hitting the rim.
# Set CONE_HEIGHT_M to None to disable this constraint.
CONE_RIM_OFFSET_M = 0.4    # meters before the target center (cone rim position)
CONE_HEIGHT_M     = HUB_HEIGHT_M + 0.20  # ball must clear this height at the rim

# =============================================================================
# --- MAPPING FUNCTIONS ---
# =============================================================================

def convert_hood_to_launch_angle(raw_hood_angle):
    """
    Convert raw hood angle (rotations) to actual launch vector angle (radians).
    0.0 rotations → 85°, 2.8 rotations → 50°.
    """
    angle_degrees = 85.0 - 12.5 * raw_hood_angle
    return np.radians(angle_degrees)


def get_slip_multiplier(rps):
    """
    Linearly interpolate ball-to-flywheel slip multiplier from RPS.
    Higher RPS → more slip → lower effective launch speed.
    """
    slope = (SLIP_AT_60_RPS - SLIP_AT_40_RPS) / (60.0 - 40.0)
    return SLIP_AT_40_RPS + slope * (rps - 40.0)


def convert_rps_to_launch_speed(rps):
    """
    Convert flywheel motor RPS to ball launch speed (m/s), accounting for
    gearing ratio and ball slip.
    """
    surface_speed = (rps * GEARING_RATIO) * (np.pi * FLYWHEEL_DIAMETER_M)
    return surface_speed * get_slip_multiplier(rps)


# =============================================================================
# --- SIMULATION ENGINE ---
# 2D drag + gravity kinematics via solve_ivp.
# =============================================================================

def simulate_flight(v0_x, v0_y, target_distance):
    """
    Simulate game piece trajectory until it reaches target_distance.
    Returns (tof, final_height, cone_clearance_height, solution)
    or (None, None, None, None) on failure.

    cone_clearance_height: ball height when it crosses x = target_distance - CONE_RIM_OFFSET_M.
    This checks whether the ball clears the cone rim before entering the basket.
    """
    A = np.pi * (BALL_DIAMETER_M / 2) ** 2
    C = 0.5 * DRAG_COEFFICIENT * AIR_DENSITY_KG_M3 * A / BALL_MASS_KG
    g = 9.81

    def derivatives(t, y):
        rx, ry, vx, vy = y
        v_mag = np.sqrt(vx**2 + vy**2)
        ax = -C * vx * v_mag
        ay = -g - C * vy * v_mag
        return [vx, vy, ax, ay]

    def hit_target_event(t, y):
        return y[0] - target_distance

    hit_target_event.terminal  = True
    hit_target_event.direction = 1

    rim_x = target_distance - CONE_RIM_OFFSET_M

    def cone_rim_event(t, y):
        # Triggers when ball's x-coordinate crosses the cone rim position
        return y[0] - rim_x

    cone_rim_event.terminal  = False  # Don't stop — just record the height
    cone_rim_event.direction = 1      # Ball moving forward

    sol = solve_ivp(
        derivatives,
        [0, 5.0],
        [0, SHOOTER_HEIGHT_M, v0_x, v0_y],
        events=[hit_target_event, cone_rim_event],
        dense_output=True,
        max_step=0.01
    )

    if len(sol.t_events[0]) > 0:
        tof          = sol.t_events[0][0]
        final_height = sol.y_events[0][0][1]
        # Cone clearance: height at rim position, or None if rim event not triggered
        if len(sol.t_events[1]) > 0:
            cone_clearance_height = sol.y_events[1][0][1]
        else:
            cone_clearance_height = None  # Ball didn't reach rim (too short)
        return tof, final_height, cone_clearance_height, sol
    return None, None, None, None


def get_final_height(rps, raw_hood, dist):
    """
    Convenience wrapper: returns (final_height, cone_clearance_height) at dist,
    or (None, None) if the ball never reaches dist.
    """
    angle = convert_hood_to_launch_angle(raw_hood)
    v0    = convert_rps_to_launch_speed(rps)
    _, height, cone_h, _ = simulate_flight(v0 * np.cos(angle), v0 * np.sin(angle), dist)
    return height, cone_h


# =============================================================================
# --- TWO-STAGE SOLVER ---
# =============================================================================

def solve_for_distance(dist):
    """
    Stage 1 — Accuracy:
        For each candidate hood angle θ, root-find v*(θ) such that:
            final_height(v*(θ), θ, dist) == HUB_HEIGHT_M
        using brentq (guaranteed convergence on bracketed sign change).

    Stage 2 — Tolerance:
        For every valid (θ, v*(θ)) pair, compute normalized finite-difference
        sensitivities of final height to perturbations in v and θ:

            sens_v     = (∂y/∂v)   / Δv
            sens_theta = (∂y/∂θ) / Δθ

        Select the pair minimizing:
            cost = W_V * sens_v² + W_THETA * sens_theta²

        This is the solution most forgiving of flywheel and hood error.
    """
    candidates = []

    for raw_hood in np.linspace(HOOD_SWEEP_MIN, HOOD_SWEEP_MAX, HOOD_SWEEP_STEPS):

        # ------------------------------------------------------------------
        # Stage 1: Root-find rps*(θ)
        # ------------------------------------------------------------------
        def height_error(rps):
            h, peak = get_final_height(rps, raw_hood, dist)
            return (h - HUB_HEIGHT_M) if h is not None else 1e6

        lo = height_error(RPS_SEARCH_MIN)
        hi = height_error(RPS_SEARCH_MAX)

        # Skip if either endpoint fails or no sign change (no root in bracket)
        if lo >= 1e6 or hi >= 1e6:
            continue
        if lo * hi >= 0:
            continue

        try:
            rps_star = brentq(height_error, RPS_SEARCH_MIN, RPS_SEARCH_MAX, xtol=0.01)
        except ValueError:
            continue

        # Sanity check: confirm the root actually hits the target
        h_check, cone_h_check = get_final_height(rps_star, raw_hood, dist)
        if h_check is None or abs(h_check - HUB_HEIGHT_M) > 0.05:
            continue

        # Enforce cone clearance constraint:
        # ball must clear CONE_HEIGHT_M at x = target_distance - CONE_RIM_OFFSET_M
        if CONE_HEIGHT_M is not None:
            if cone_h_check is None or cone_h_check < CONE_HEIGHT_M:
                continue

        # ------------------------------------------------------------------
        # Stage 2: Normalized finite-difference sensitivities
        # ------------------------------------------------------------------

        # ∂y/∂v — perturb flywheel RPS, hold hood fixed
        h_vp, _ = get_final_height(rps_star + DELTA_V, raw_hood, dist)
        h_vm, _ = get_final_height(rps_star - DELTA_V, raw_hood, dist)
        if h_vp is None or h_vm is None:
            continue
        dy_dv = (h_vp - h_vm) / (2 * DELTA_V)

        # ∂y/∂θ — perturb hood angle, hold RPS fixed
        h_tp, _ = get_final_height(rps_star, raw_hood + DELTA_THETA, dist)
        h_tm, _ = get_final_height(rps_star, raw_hood - DELTA_THETA, dist)
        if h_tp is None or h_tm is None:
            continue
        dy_dtheta = (h_tp - h_tm) / (2 * DELTA_THETA)

        # Normalize by step size to be scale-independent
        norm_sens_v     = dy_dv     / DELTA_V
        norm_sens_theta = dy_dtheta / DELTA_THETA

        cost = W_V * norm_sens_v ** 2 + W_THETA * norm_sens_theta ** 2

        # Compute ToF and cone clearance height for the selected solution
        angle = convert_hood_to_launch_angle(raw_hood)
        v0    = convert_rps_to_launch_speed(rps_star)
        tof, final_height, cone_h, sol = simulate_flight(
            v0 * np.cos(angle), v0 * np.sin(angle), dist
        )

        candidates.append({
            'cost':       cost,
            'hood':       raw_hood,
            'rps':        rps_star,
            'tof':        tof,
            'height':     final_height,
            'rim_height': cone_h,   # ball height at x = dist - CONE_RIM_OFFSET_M
            'sol':        sol,
            'dy_dv':      dy_dv,
            'dy_dtheta':  dy_dtheta,
        })

    if not candidates:
        return None

    candidates.sort(key=lambda x: x['cost'])
    return candidates[0]


# =============================================================================
# --- VERIFY MODE ---
# =============================================================================

def run_verify():
    print("\n  Confirming hand-tuned data points...\n")
    cone_col = f"@{dist:.2f}-{CONE_RIM_OFFSET_M:.1f}m" if False else f"RimH@-{CONE_RIM_OFFSET_M:.1f}m"
    print(f"  {'Distance (m)':>12} | {'ToF (s)':>7} | {'Height (m)':>10} | {cone_col:>12} | {'Match?':>15}")
    print("  " + "-" * 70)

    plt.figure(figsize=(10, 6))
    plt.title("Verification Trajectories (Hand-Tuned)")
    plt.xlabel("Distance (m)")
    plt.ylabel("Height (m)")
    plt.axhline(y=HUB_HEIGHT_M, color='r', linestyle='--', label="Hub Height")
    if CONE_HEIGHT_M is not None:
        plt.axhline(y=CONE_HEIGHT_M, color='orange', linestyle=':',
                    label=f"Cone Rim ({CONE_HEIGHT_M:.2f}m @ -{CONE_RIM_OFFSET_M:.1f}m)")

    java_buf = []
    for dist, raw_hood, rps in zip(TARGET_DISTANCES_M, HOOD_ANGLES_RAW, FLYWHEEL_RPS_RAW):
        angle = convert_hood_to_launch_angle(raw_hood)
        v0    = convert_rps_to_launch_speed(rps)
        tof, height, cone_h, sol = simulate_flight(v0 * np.cos(angle), v0 * np.sin(angle), dist)

        if tof is not None:
            match    = "✓" if abs(height - HUB_HEIGHT_M) < 0.1 else f"MISS {abs(height - HUB_HEIGHT_M):.2f}m"
            cone_str = f"{cone_h:.3f}m" if cone_h is not None else "  N/A"
            clears   = " ✓" if (cone_h is not None and (CONE_HEIGHT_M is None or cone_h >= CONE_HEIGHT_M)) else " ✗ RIM"
            print(f"  {dist:12.2f} | {tof:7.3f} | {height:10.3f} | {cone_str+clears:>12} | {match:>15}")
            java_buf.append(f"        tofTreeMap.put({dist:.2f}, {tof:.3f});")
            ts = np.linspace(0, tof, 100)
            state = sol.sol(ts)
            plt.plot(state[0], state[1], label=f"{dist}m")
        else:
            print(f"  {dist:12.2f} | {'FAILED':>7}")

    print("\n  === tofTreeMap ===")
    for line in java_buf:
        print(f"  {line}")

    plt.legend()
    plt.grid(True)
    plt.xlim(left=0)
    plt.ylim(bottom=0)
    plt.show()


# =============================================================================
# --- SOLVE MODE ---
# =============================================================================

def run_solve():
    cone_str = f"{CONE_HEIGHT_M:.2f}m @ -{CONE_RIM_OFFSET_M:.1f}m before target" if CONE_HEIGHT_M is not None else "none"
    print(f"\n  Sweeping {HOOD_SWEEP_STEPS} hood angles per distance  |  Cone clearance: {cone_str}\n")
    header = f"  {'Dist':>5} | {'Hood':>7} | {'RPS':>7} | {'ToF':>6} | {'Hit H':>6} | {'Rim H':>7} | {'∂y/∂v':>8} | {'∂y/∂θ':>8} | {'Cost':>10}"
    print(header)
    print("  " + "-" * (len(header) - 2))

    tof_buf  = []
    rps_buf  = []
    hood_buf = []

    plt.figure(figsize=(10, 6))
    plt.title("Solved Optimal Trajectories (Minimum Sensitivity)")
    plt.xlabel("Distance (m)")
    plt.ylabel("Height (m)")
    plt.axhline(y=HUB_HEIGHT_M, color='r', linestyle='--', label="Hub Height")
    if CONE_HEIGHT_M is not None:
        plt.axhline(y=CONE_HEIGHT_M, color='orange', linestyle=':',
                    label=f"Cone Rim ({CONE_HEIGHT_M:.2f}m @ -{CONE_RIM_OFFSET_M:.1f}m)")

    for dist in SOLVER_DISTANCES_M:
        r = solve_for_distance(dist)
        if r is None:
            print(f"  {dist:5.2f} | NO SOLUTION FOUND (try lowering CONE_HEIGHT_M or expanding hood sweep)")
            continue

        rim_h_str = f"{r['rim_height']:.3f}" if r['rim_height'] is not None else "  N/A "
        print(
            f"  {dist:5.2f} | {r['hood']:7.3f} | {r['rps']:7.2f} | "
            f"{r['tof']:6.3f} | {r['height']:6.3f} | {rim_h_str:>7} | "
            f"{r['dy_dv']:8.4f} | {r['dy_dtheta']:8.4f} | {r['cost']:10.4f}"
        )

        tof_buf.append( f"        tofTreeMap.put({dist:.2f}, {r['tof']:.3f});")
        rps_buf.append( f"        lowerWheelTreeMap.put({dist:.2f}, {r['rps']:.1f});")
        hood_buf.append(f"        hoodTreeMap.put({dist:.2f}, {r['hood']:.3f});")

        sol = r['sol']
        ts  = np.linspace(0, r['tof'], 100)
        state = sol.sol(ts)
        plt.plot(state[0], state[1],
                 label=f"{dist}m  hood={r['hood']:.2f} rot, {r['rps']:.1f} RPS")

    print("\n  =========================================================")
    print("               GENERATED TREEMAP CODE                    ")
    print("  =========================================================")
    print("  // tofTreeMap")
    for l in tof_buf:
        print(f"  {l}")
    print("  // lowerWheelTreeMap")
    for l in rps_buf:
        print(f"  {l}")
    print("  // hoodTreeMap")
    for l in hood_buf:
        print(f"  {l}")
    print("\n  * Paste this into frc2025/config/ShooterTreeMap.java *")

    plt.legend()
    plt.grid(True)
    plt.xlim(left=0)
    plt.ylim(bottom=0)
    plt.show()


# =============================================================================
# --- MAIN ---
# =============================================================================

def main():
    print("=========================================================")
    print("              FRC SHOOTER SIMULATOR                      ")
    print(f"              MODE: {MODE.upper():<38}")
    print("=========================================================")

    if MODE == "verify":
        run_verify()
    elif MODE == "solve":
        run_solve()
    else:
        print(f"  Unknown MODE: '{MODE}'. Set MODE to 'verify' or 'solve'.")


if __name__ == '__main__':
    main()
