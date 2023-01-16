import numpy as np
from agent_class import Agent
from robot_class import Robot


def circle(uav: Agent):
    """
    Method called at each iteration of the program if the circle mode is active
    (every time a new packet is received from QTM -> ~ every 0.02s)
    Generates the coordinates (x, y, z, and yaw) of the next position to reach on the circle
    """

    t = uav.circle_t  # (s)

    # ------------------ A remplir --------------------- #
    #
    #
    targeted_x = 0  # (m)
    targeted_y = 0  # (m)
    targeted_z = 0  # (m)
    targeted_yaw = 0  # (rad)
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


def circle_tangent_x_axis(uav: Agent):
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
    targeted_x = 0  # (m)
    targeted_y = 0  # (m)
    targeted_z = 0  # (m)
    targeted_yaw = 0  # (rad)
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


def point_of_interest(uav: Agent, target: Robot):
    """
    Method called at each iteration of the program if the point_of_interest mode is active
    (every time a new packet is received from QTM -> ~ every 0.02s)
    Generates the coordinates (x, y, z, and yaw) of the next position to reach on the circle
    """

    t = uav.circle_t  # (s)
    robot_x = target.extpos.x  # (m)
    robot_y = target.extpos.y  # (m)
    # x = uav.extpos.x            # (m)
    # y = uav.extpos.y            # (m)
    # vx = uav.velocity[0]        # (m/s)
    # vy = uav.velocity[1]        # (m/s)

    # ------------------ A remplir --------------------- #
    #
    #
    targeted_x = 0  # (m)
    targeted_y = 0  # (m)
    targeted_z = 0  # (m)
    targeted_yaw = 0  # (rad)
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


def control_law(uav: Agent, targeted_x: float, targeted_y: float, targeted_z: float, targeted_yaw: float):
    """
    Generates the command input (roll angle, pitch angle, yaw angle, thrust) that will be forwarded to the UAV,
    based on the UAV current state and its targeted position
    """

    delta_t = uav.delta_t  # (s)
    measured_x = uav.extpos.x  # (m)
    measured_y = uav.extpos.y  # (m)
    measured_z = uav.extpos.z  # (m)
    measured_yaw = uav.yaw * np.pi / 180  # (rad)
    measured_vx = uav.velocity[0]  # (m/s)
    measured_vy = uav.velocity[1]  # (m/s)
    measured_vz = uav.velocity[2]  # (m/s)
    previous_iz = uav.previous_iz  # (PWM)

    # ------------------ A remplir --------------------- #
    #
    #
    pitch = 0  # (rad)
    roll = 0  # (rad)
    #
    #
    # -------------------------------------------------- #

    # -- Yaw control law -- #
    yaw_kp = 10
    targeted_yaw = targeted_yaw % (2 * np.pi)
    if targeted_yaw > np.pi:
        targeted_yaw = targeted_yaw - (2 * np.pi)

    measured_yaw = measured_yaw % (2 * np.pi)
    if measured_yaw > np.pi:
        measured_yaw = measured_yaw - (2 * np.pi)

    yaw_error_0 = targeted_yaw - measured_yaw
    yaw_error_1 = targeted_yaw - measured_yaw + 2 * np.pi
    yaw_error_2 = targeted_yaw - measured_yaw - 2 * np.pi

    if abs(yaw_error_1) < abs(yaw_error_0) and abs(yaw_error_1) < abs(yaw_error_2):
        yaw_error = yaw_error_1
    elif abs(yaw_error_2) < abs(yaw_error_0) and abs(yaw_error_2) < abs(yaw_error_1):
        yaw_error = yaw_error_2
    else:
        yaw_error = yaw_error_0

    yaw_rate = - round(yaw_kp * yaw_error * 180 / np.pi)  # (°/s)

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
    roll = roll * 180 / np.pi
    pitch = pitch * 180 / np.pi

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
                             measured_x, measured_y, measured_z, measured_yaw * 180 / np.pi,
                             measured_vx, measured_vy, measured_vz,
                             targeted_x, targeted_y, targeted_z, targeted_yaw * 180 / np.pi,
                             roll, pitch, yaw_rate, thrust])

    # -- Prepare for next iteration -- #
    uav.previous_iz = iz

    return roll, pitch, yaw_rate, thrust
