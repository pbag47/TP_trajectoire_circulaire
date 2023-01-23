import numpy
from agent_class import Agent
from robot_class import Robot


def circle(uav: Agent) -> (float, float, float, int, float, float):
    """
    Method called at each iteration of the program if the circle mode is active
    (every time a new packet is received from QTM -> ~ every 0.02s)
    Generates the coordinates (x, y, z, and yaw) of the next position to reach on the circle
    """

    t = uav.circle_t  # (s)

    # ------------------ A remplir --------------------- #
    #
    #
    # -- Parameters -- #
    radius = 0.5  # (m)
    period = 2 * numpy.pi  # (s)
    frequency = 1 / period  # (Hz)
    omega = 2 * numpy.pi * frequency  # (rad/s)
    #
    # -- Target point -- #
    targeted_x = radius * numpy.cos(omega * t)  # (m)
    targeted_y = radius * numpy.sin(omega * t)  # (m)
    targeted_z = 0.5  # (m)
    targeted_yaw = 0  # (°)
    #
    #
    # -------------------------------------------------- #

    try:
        roll, pitch, yaw_rate, thrust = control_law(uav, targeted_x, targeted_y, targeted_z, targeted_yaw)
        return roll, pitch, yaw_rate, thrust, targeted_x, targeted_y
    except Exception as e:
        print(' -- Warning -- Error |', e, '| detected in uav_control_law.control_law() function,'
                                           ' switching to Standby state')
        uav.standby()


def circle_tangent_x_axis(uav: Agent) -> (float, float, float, int, float, float):
    """
    Method called at each iteration of the program if the circle_with_tangent_x_axis mode is active
    (every time a new packet is received from QTM -> ~ every 0.02s)
    Generates the coordinates (x, y, z, and yaw) of the next position to reach on the circle
    """

    t = uav.circle_t  # (s)
    # vx = uav.velocity[0]    # (m/s)
    # vy = uav.velocity[1]    # (m/s)

    # ------------------ A remplir --------------------- #
    #
    #
    # -- Parameters -- #
    radius = 0.5  # (m)
    period = 2 * numpy.pi  # (s)
    frequency = 1 / period  # (Hz)
    omega = 2 * numpy.pi * frequency  # (rad/s)
    #
    # -- Target point -- #
    targeted_x = radius * numpy.cos(omega * t)  # (m)
    targeted_y = radius * numpy.sin(omega * t)  # (m)
    targeted_z = 0.5  # (m)
    #
    #
    # targeted_yaw = - np.arctan2(vx, vy) + np.pi/2   # (rad)
    targeted_yaw = omega * t + numpy.pi / 2  # (rad)
    #
    #
    # -------------------------------------------------- #

    try:
        roll, pitch, yaw_rate, thrust = control_law(uav, targeted_x, targeted_y, targeted_z, targeted_yaw)
        return roll, pitch, yaw_rate, thrust, targeted_x, targeted_y
    except Exception as e:
        print(' -- Warning -- Error |', e, '| detected in uav_control_law.control_law() function,'
                                           ' switching to Standby state')
        uav.standby()


def point_of_interest(uav: Agent, target: Robot) -> (float, float, float, int, float, float):
    """
    Method called at each iteration of the program if the point_of_interest mode is active
    (every time a new packet is received from QTM -> ~ every 0.02s)
    Generates the coordinates (x, y, z, and yaw) of the next position to reach on the circle
    """

    t = uav.circle_t  # (s)
    robot_x = target.extpos.x  # (m)
    robot_y = target.extpos.y  # (m)
    x = uav.extpos.x  # (m)
    y = uav.extpos.y  # (m)
    # vx = uav.velocity[0]        # (m/s)
    # vy = uav.velocity[1]        # (m/s)

    # ------------------ A remplir --------------------- #
    #
    #
    # -- Parameters -- #
    radius = 0.5  # (m)
    period = 2 * numpy.pi  # (s)
    frequency = 1 / period  # (Hz)
    omega = 2 * numpy.pi * frequency  # (rad/s)
    #
    # -- Target point -- #
    targeted_x = robot_x + radius * numpy.cos(omega * t)  # (m)
    targeted_y = robot_y + radius * numpy.sin(omega * t)  # (m)
    targeted_z = 0.5  # (m)
    #
    targeted_yaw = - numpy.arctan2(robot_x - x, robot_y - y) + numpy.pi / 2  # (rad)
    # targeted_yaw = - np.arctan2(vx, vy) + np.pi   # (rad)
    # targeted_yaw = omega * t + np.pi  # (rad)
    #
    #
    # -------------------------------------------------- #

    try:
        roll, pitch, yaw_rate, thrust = control_law(uav, targeted_x, targeted_y, targeted_z, targeted_yaw)
        return roll, pitch, yaw_rate, thrust, targeted_x, targeted_y
    except Exception as e:
        print(' -- Warning -- Error |', e, '| detected in uav_control_law.control_law() function,'
                                           ' switching to Standby state')
        uav.standby()


def control_law(uav: Agent,
                targeted_x: float,
                targeted_y: float,
                targeted_z: float,
                targeted_yaw: float) -> (float, float, float, int):
    """
    Generates the command input (roll angle, pitch angle, yaw angle, thrust) that will be forwarded to the UAV,
    based on the UAV current state and its targeted position
    """

    delta_t = uav.delta_t  # (s)
    measured_x = uav.extpos.x  # (m)
    measured_y = uav.extpos.y  # (m)
    measured_z = uav.extpos.z  # (m)
    measured_yaw = uav.yaw * numpy.pi / 180  # (rad)
    measured_vx = uav.velocity[0]  # (m/s)
    measured_vy = uav.velocity[1]  # (m/s)
    measured_vz = uav.velocity[2]  # (m/s)
    previous_iz = uav.previous_iz  # (PWM)

    # ------------------ A remplir --------------------- #
    #
    #
    # -- Parameters -- #
    xy_kp = 1
    xy_kd = 0.4
    #
    # -- Errors -- #
    x_error = targeted_x - measured_x
    y_error = targeted_y - measured_y
    #
    # -- Coordinates system change (rotation matrix) -- #
    xn_error = x_error * numpy.cos(measured_yaw) + y_error * numpy.sin(measured_yaw)
    yn_error = - x_error * numpy.sin(measured_yaw) + y_error * numpy.cos(measured_yaw)
    xn_velocity = measured_vx * numpy.cos(measured_yaw) + measured_vy * numpy.sin(measured_yaw)
    yn_velocity = - measured_vx * numpy.sin(measured_yaw) + measured_vy * numpy.cos(measured_yaw)
    #
    # -- Pitch control law -- #
    px = xy_kp * xn_error
    dx = - xn_velocity
    pitch = xy_kd * (px + dx)
    #
    # -- Roll control law -- #
    py = xy_kp * yn_error
    dy = - yn_velocity
    roll = - xy_kd * (py + dy)
    #
    #
    # -------------------------------------------------- #

    # -- Yaw control law -- #
    yaw_kp = 5
    targeted_yaw = targeted_yaw % (2 * numpy.pi)
    if targeted_yaw > numpy.pi:
        targeted_yaw = targeted_yaw - (2 * numpy.pi)

    measured_yaw = measured_yaw % (2 * numpy.pi)
    if measured_yaw > numpy.pi:
        measured_yaw = measured_yaw - (2 * numpy.pi)

    yaw_error_0 = targeted_yaw - measured_yaw
    yaw_error_1 = targeted_yaw - measured_yaw + 2 * numpy.pi
    yaw_error_2 = targeted_yaw - measured_yaw - 2 * numpy.pi

    if abs(yaw_error_1) < abs(yaw_error_0) and abs(yaw_error_1) < abs(yaw_error_2):
        yaw_error = yaw_error_1
    elif abs(yaw_error_2) < abs(yaw_error_0) and abs(yaw_error_2) < abs(yaw_error_1):
        yaw_error = yaw_error_2
    else:
        yaw_error = yaw_error_0

    yaw_rate = - round(yaw_kp * yaw_error * 180 / numpy.pi)  # (°/s)

    # -- Height control law -- #
    z_kp = 32500
    z_ki = 8125
    z_kd = 16250
    thrust_at_steady_state = 38000

    z_error = targeted_z - measured_z
    pz = z_kp * z_error
    iz = previous_iz + z_ki * z_error * delta_t
    dz = - z_kd * measured_vz
    thrust = thrust_at_steady_state + pz + iz + dz

    # -- Roll and Pitch units conversion (rad to °) -- #
    roll = roll * 180 / numpy.pi
    pitch = pitch * 180 / numpy.pi

    # -- Attitude saturation check -- #
    max_roll = 20  # (°)
    max_pitch = 20  # (°)
    max_yaw_rate = 180  # (°/s)

    thrust = round(thrust)
    if thrust > 65000:
        thrust = 65000
    elif thrust < 0:
        thrust = 0

    if roll > max_roll:
        roll = max_roll
    elif roll < -max_roll:
        roll = -max_roll

    if pitch > max_pitch:
        pitch = max_pitch
    elif pitch < -max_pitch:
        pitch = -max_pitch

    if yaw_rate > max_yaw_rate:
        yaw_rate = max_yaw_rate
    elif yaw_rate < -max_yaw_rate:
        yaw_rate = -max_yaw_rate

    # -- Log flight data -- #
    uav.csv_logger.writerow([uav.name, uav.timestamp,
                             measured_x, measured_y, measured_z, measured_yaw * 180 / numpy.pi,
                             measured_vx, measured_vy, measured_vz,
                             targeted_x, targeted_y, targeted_z, targeted_yaw * 180 / numpy.pi,
                             roll, pitch, yaw_rate, thrust])

    # -- Prepare for next iteration -- #
    uav.previous_iz = iz

    return roll, pitch, yaw_rate, thrust
