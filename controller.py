wind_active = False  # Select whether you want to activate wind or not
group_number = 22  # Enter your group number here

# Physical constants
g = 9.81       # Gravitational acceleration (m/s^2)
m = 1.2665675038538184  # mass of the drone (kg)
ixx = 0.2850921475190987  # moment of inertia around the x-axis (kg*m^2)
L = 0.25  # Distance between the drone's rotors (m)

# Integral control accumulators for error integration
int_err_y = 0   
int_err_x = 0   
int_err_at = 0  
prev_target_pos = [100, 100]  # Invalid target position to trigger reset of integral accumulators


def disturbance_observer(err_x, v_x, v_y):
    """
    Estimate disturbances using a disturbance observer. 
    By integrating this estimation into the control loop, 
    the controller effectively mitigates disturbances,
    minimizing their impact on system performance. 
    In simulations with wind, the y-axis remains largely unaffected, 
    while the x-axis is significantly influenced.

    Args:
    err_x (float): Current position error along the x-axis
    v_x (float): Current velocity along the x-axis
    v_y (float): Current velocity along the y-axis
    dob_alpha (float): Adaptive gain for x-axis disturbance estimation
    dob_beta (float): Fixed gain for y-axis disturbance estimation

    Returns:
    tuple: Estimated disturbances (wind_speed_x, wind_speed_y)
    """
    dob_beta = 0.00001
    dob_alpha = 2 if abs(err_x) < 0.35 else 0.8
    wind_speed_x = dob_alpha * v_x
    wind_speed_y = dob_beta * v_y
    return wind_speed_x, wind_speed_y

def dynamic_pid_at(error_x, error_y, int_err_x, int_err_y, dt):
    """
    Select PID coefficients for attitude control based on error_x and update integral accumulator.
    For small errors (<1), fine-tuning and precision are prioritized, 
    while larger errors (>1) prompt faster correction and stabilization

    Args:
    error_x (float): Error along the x-axis
    int_err_x (float): Integral of the x-axis position error
    dt (float): Time step for integral calculation

    Returns:
    tuple: PID coefficients (Kpat, Kdat, Kiat) and updated integral accumulator (int_err_x)
    """
    if abs(error_x) < 1:
        Kpat, Kdat, Kiat = 166, 100, 66
        int_err_x = min(max(int_err_x + error_x * dt, -0.1), 0.1)
    elif abs(error_x) < 0.35:
        Kpat, Kdat, Kiat = 300, 400, 66
    else:
        Kpat, Kdat, Kiat = 30, 40, 1
        int_err_x = min(max(int_err_x + error_x * dt, -0.5), 0.5)
    
    # Update integral accumulators with clamping to prevent windup
    int_err_y = min(max(int_err_y + error_y * dt, -0.1), 0.1)
        
    return Kpat, Kdat, Kiat, int_err_x, int_err_y

# Implement a controller
def controller(state, target_position, dt):
    # state format: [position_x (m), position_y (m), velocity_x (m/s), velocity_y (m/s), attitude(radians), angular_velocity (rad/s)]
    # target_pos format: [x (m), y (m)]
    # dt: time step
    # return: action format: (u_1, u_2)
    # u_1 and u_2 are the throttle settings of the left and right motor

    global int_err_y, int_err_x, int_err_at, prev_target_pos

    # Unpack current state and target position
    p_x, p_y, v_x, v_y, at, ang_v = state
    f_x, f_y = target_position

    # Integral Cleanup, reset integral accumulators if target position changes
    if target_position != prev_target_pos:
        int_err_y = int_err_x = int_err_at = 0
        prev_target_pos = target_position

    # Calculate positional errors
    error_y = p_y - f_y
    error_x = p_x - f_x

    # Calculate disturbances using disturbance observer
    wind_speed_x, wind_speed_y = disturbance_observer(error_x, v_x, v_y)

    # PID coefficients
    Kpy, Kdy, Kiy = 60, 50, 90
    Kpx, Kdx, Kix = 1.0, 1.7, 3

    Kpat, Kdat, Kiat, int_err_x, int_err_y = dynamic_pid_at(error_x, error_y, int_err_x, int_err_y, dt)

    # Calculate target at (phi_c) based on x-axis control
    target_at = -1 / g * (Kdx * (v_x + wind_speed_x) + Kpx * error_x + Kix * int_err_x)
    error_at = at - target_at  

    # Update integral accumulator for phi
    int_err_at = min(max(int_err_at + error_at * dt, -1), 1)

    # Calculate upward thrust needed based on PID outputs
    F = m * (g + Kdy * (v_y + wind_speed_y) + Kpy * error_y + Kiy * int_err_y)

    # Calculate torque needed based on PID outputs for phi
    M = ixx * (Kdat * ang_v + Kpat * error_at + Kiat * int_err_at)

    # Calculate motor commands using mixer theory with clamping to prevent actuator saturation
    u1 = 0.5 * (F - M / L)
    u2 = 0.5 * (F + M / L)
    u1_clamped = min(max(0, u1), 1)
    u2_clamped = min(max(0, u2), 1)
    action=(u1_clamped, u2_clamped)

    return action
