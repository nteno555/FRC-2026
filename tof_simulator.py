import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# =============================================================================
# --- CONFIGURABLE PHYSICAL CONSTANTS ---
# Modifying these variables will physically change the flight trajectories. 
# =============================================================================

# Game Piece Physics 
BALL_MASS_KG = 0.20     
BALL_DIAMETER_M = 0.15 
AIR_DENSITY_KG_M3 = 1.204 
DRAG_COEFFICIENT = 0.47   

# Robot & Field Geometry
SHOOTER_HEIGHT_M = 0.56    # Height of shooter exit from floor
HUB_HEIGHT_M = 1.44       # Height of target center from floor
FLYWHEEL_DIAMETER_M = 0.1016 # e.g. 4-inch wheels
GEARING_RATIO = 0.5       # Flywheel motor to wheel ratio 
SLIP_AT_40_RPS = 0.90     # Multiplier at 40 RPS (less slip at low speed)
SLIP_AT_60_RPS = 0.90    # Multiplier at 60 RPS (more slip at high speed)

# Tuned Robot Map Data
TARGET_DISTANCES_M = [1.29, 2.47, 3.07, 4.04, 5.24]
HOOD_ANGLES_RAW = [0.8, 1.7, 2.0, 2.4, 2.4]  # From hoodTreeMap
FLYWHEEL_RPS_RAW = [40.0, 43.0, 45.0, 50.0, 58.0]  # From lowerWheelTreeMap

# =============================================================================
# --- MAPPING FUNCTIONS ---
# Modify these to correctly map your raw robot units to standard physical units
# =============================================================================

def convert_hood_to_launch_angle(raw_hood_angle):
    """
    Convert raw hood angle from the TreeMap to the actual launch vector angle.
    Returns: angle in RADIANS from the horizontal floor.
    """
    # 0 rotations = 85 degrees, 2.8 rotations = 50 degrees
    # m = (50 - 85) / 2.8 = -35 / 2.8 = -12.5 degrees / rotation
    angle_degrees = 85.0 - 12.5 * raw_hood_angle
    return np.radians(angle_degrees)

def get_slip_multiplier(rps):
    """
    Linearly interpolates the slip multiplier based on the given RPS.
    Slipping generally increases (multiplier decreases) at higher RPS.
    """
    slope = (SLIP_AT_60_RPS - SLIP_AT_40_RPS) / (60.0 - 40.0)
    return SLIP_AT_40_RPS + slope * (rps - 40.0)

def convert_rps_to_launch_speed(rps):
    """
    Convert the setpoint flywheel RPS to actual game piece launch speed.
    Returns: launch speed in meters per second (m/s).
    """
    surface_speed = (rps * GEARING_RATIO) * (np.pi * FLYWHEEL_DIAMETER_M)
    current_slip = get_slip_multiplier(rps)
    return surface_speed * current_slip

# =============================================================================
# --- SIMULATION ENGINE ---
# Uses 2D Drag Kinematics to find the Time-Of-Flight
# =============================================================================
def simulate_flight(v0_x, v0_y, target_distance):
    A = np.pi * (BALL_DIAMETER_M / 2)**2
    C = 0.5 * DRAG_COEFFICIENT * AIR_DENSITY_KG_M3 * A / BALL_MASS_KG
    g = 9.81
    
    def derivatives(t, y):
        rx, ry, vx, vy = y
        v_mag = np.sqrt(vx**2 + vy**2)
        ax = -C * vx * v_mag
        ay = -g - C * vy * v_mag

        # ax = 0
        # ay = -g
        return [vx, vy, ax, ay]
        
    def hit_target_event(t, y):
        # Trigger when x position explicitly crosses target_distance
        return y[0] - target_distance
        
    hit_target_event.terminal = True
    hit_target_event.direction = 1
    
    y0 = [0, SHOOTER_HEIGHT_M, v0_x, v0_y]


    print("solve_ivp", v0_x, v0_y) 
    sol = solve_ivp(
        derivatives, 
        [0, 5.0],  # Give it up to 5 seconds to hit
        y0, 
        events=hit_target_event,
        dense_output=True,
        max_step=0.01
    )
    
    if len(sol.t_events[0]) > 0:
        tof = sol.t_events[0][0]
        final_height = sol.y_events[0][0][1]
        return tof, final_height, sol
    else:
        return None, None, None

def main():
    print("=========================================================")
    print("             FRC TIME-OF-FLIGHT SIMULATOR                ")
    print("=========================================================")
    print("Simulating interpolator points with drag + gravity...\n")
    
    print(f"{'Distance (m)':>12} | {'ToF (s)':>8} | {'Final Height (m)':>17} | {'Target Match?':>15}")
    print("-" * 65)
    
    java_code_buffer = []
    
    plt.figure(figsize=(10, 6))
    plt.title("Game Piece Trajectories")
    plt.xlabel("Distance (m)")
    plt.ylabel("Height (m)")
    plt.axhline(y=HUB_HEIGHT_M, color='r', linestyle='--', label="Hub Height")
    plt.axvline(x=0, color='k', alpha=0.3)
    plt.axhline(y=0, color='k', alpha=0.3)
    
    for dist, raw_hood, rps in zip(TARGET_DISTANCES_M, HOOD_ANGLES_RAW, FLYWHEEL_RPS_RAW):
        angle = convert_hood_to_launch_angle(raw_hood)
        v0 = convert_rps_to_launch_speed(rps)
        
        v0_x = v0 * np.cos(angle)
        v0_y = v0 * np.sin(angle)
        
        tof, height, sol = simulate_flight(v0_x, v0_y, dist)
        
        if tof is not None:
            # Check if calculated height roughly matches the expected hub height
            height_match = "YES" if abs(height - HUB_HEIGHT_M) < 0.5 else f"NO (miss by {abs(height-HUB_HEIGHT_M):.2f}m)"
            print(f"{dist:12.2f} | {tof:8.3f} | {height:17.2f} | {height_match:>15}")
            
            java_code_buffer.append(f"        tofTreeMap.put({dist:.2f}, {tof:.3f});")
            
            # Plot the trajectory
            ts = np.linspace(0, tof, 100)
            state = sol.sol(ts)
            plt.plot(state[0], state[1], label=f"Dist: {dist}m, ToF: {tof:.2f}s")
        else:
            print(f"{dist:12.2f} | {'FAILED':>8} | {'BALL FELL SHORT':>17} | {'NO':>15}")

            
    print("\n=========================================================")
    print("                 GENERATED TREEMAP CODE                  ")
    print("=========================================================")
    for line in java_code_buffer:
        print(line)
        
    print("\n* Paste this into frc2025/config/ShooterTreeMap.java *")
    
    plt.legend()
    plt.grid(True)
    plt.xlim(left=0)
    plt.ylim(bottom=0)
    plt.show()

if __name__ == '__main__':
    main()
