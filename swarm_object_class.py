import logging
import numpy
import uav_control_law

from agent_class import Agent
from flight_state_class import FlightState
from robot_class import Robot
from typing import List


logger = logging.getLogger(__name__)


class SwarmObject:
    def __init__(self):
        # ---- Parameters ---- #
        # Distance tolerance around a waypoint for it to be considered as reached (m)
        self.distance_to_waypoint_threshold: float = 0.05

        # ---- Attributes initialization ---- #
        self.manual_x: float = 0.0
        self.manual_y: float = 0.0
        self.manual_z: float = 0.0
        self.manual_yaw: float = 0.0
        self.manual_flight_agents_list: List[str] = []

        self.ready_to_fly: bool = False
        self.swarm_agent_list: List[Agent] = []
        self.robot_list: List[Robot] = []

    def add_agent(self, agent: Agent):
        self.swarm_agent_list.append(agent)

    def flight_sequence(self):
        if self.ready_to_fly:
            for agent in self.swarm_agent_list:
                if agent.enabled and not agent.state == FlightState.NOT_FLYING:
                    if agent.state == FlightState.STANDBY:
                        self.standby_control_law(agent)

                    if agent.state == FlightState.TAKEOFF:
                        self.takeoff_control_law(agent)

                    if agent.state == FlightState.LAND:
                        self.landing_control_law(agent)

                    if agent.state == FlightState.MANUAL_FLIGHT:
                        self.manual_control_law(agent)

                    if agent.state == FlightState.CIRCLE:
                        self.circle(agent)

                    if agent.state == FlightState.CIRCLE_TGX:
                        self.circle_tangent_x_axis(agent)

                    if agent.state == FlightState.POI:
                        self.point_of_interest(agent)
                else:
                    agent.cf.commander.send_stop_setpoint()
        else:
            if all([agent.battery_test_passed for agent in self.swarm_agent_list]):
                self.ready_to_fly = True
                print('UAV connection recap :')
                for agent in self.swarm_agent_list:
                    if distance([agent.position.x, agent.position.y, agent.position.z], agent.initial_position) > 0.5:
                        agent.stop()
                    agent.setup_finished = True
                    print('    - <', agent.name, '> :')
                    if agent.position.x >= 0:
                        print('        x = ', round(agent.position.x, 3), 'm')
                    else:
                        print('        x =', round(agent.position.x, 3), 'm')
                    if agent.position.y >= 0:
                        print('        y = ', round(agent.position.y, 3), 'm')
                    else:
                        print('        y =', round(agent.position.y, 3), 'm')
                    if agent.position.z >= 0:
                        print('        z = ', round(agent.position.z, 3), 'm')
                    else:
                        print('        z =', round(agent.position.z, 3), 'm')
                    print('        Battery level =', round(agent.initial_battery_level), '%')
                    if agent.enabled:
                        print('        Flight enabled')
                    else:
                        print('        -- Warning -- : Flight disabled')

    def manual_control_law(self, agent: Agent):
        kp = 0.50
        ks = 0.15
        if self.manual_x >= 0:
            agent.standby_position[0] = agent.position.x - kp * numpy.sqrt(self.manual_x)
        else:
            agent.standby_position[0] = agent.position.x + kp * numpy.sqrt(-self.manual_x)

        if self.manual_y >= 0:
            agent.standby_position[1] = agent.position.y - kp * numpy.sqrt(self.manual_y)
        else:
            agent.standby_position[1] = agent.position.y + kp * numpy.sqrt(-self.manual_y)

        agent.standby_position[2] = self.manual_z

        if agent.standby_position[0] < agent.x_boundaries[0] + ks:
            agent.standby_position[0] = agent.x_boundaries[0] + ks
        if agent.standby_position[0] > agent.x_boundaries[1] - ks:
            agent.standby_position[0] = agent.x_boundaries[1] - ks
        if agent.standby_position[1] < agent.y_boundaries[0] + ks:
            agent.standby_position[1] = agent.y_boundaries[0] + ks
        if agent.standby_position[1] > agent.y_boundaries[1] - ks:
            agent.standby_position[1] = agent.y_boundaries[1] - ks
        if agent.standby_position[2] < agent.z_boundaries[0] + ks:
            agent.standby_position[2] = agent.z_boundaries[0] + ks
        if agent.standby_position[2] > agent.z_boundaries[1] - ks:
            agent.standby_position[2] = agent.z_boundaries[1] - ks

        agent.standby_yaw = self.manual_yaw

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   agent.standby_position[0], agent.standby_position[1],
                                   agent.standby_position[2], agent.standby_yaw,
                                   'None', 'None', 'None', 'None'])

        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

    def takeoff_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.takeoff_position[0], agent.takeoff_position[1],
                                                  agent.takeoff_position[2], agent.takeoff_yaw)

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   agent.takeoff_position[0], agent.takeoff_position[1],
                                   agent.takeoff_position[2], agent.takeoff_yaw,
                                   'None', 'None', 'None', 'None'])

        d = distance([agent.position.x, agent.position.y,
                      agent.position.z], agent.takeoff_position)
        if d <= self.distance_to_waypoint_threshold:
            logger.info(agent.name + ' Takeoff completed')
            agent.standby()
            agent.standby_position[2] = agent.takeoff_height

    def landing_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.land_position[0], agent.land_position[1],
                                                  agent.land_position[2], agent.land_yaw)

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   agent.land_position[0], agent.land_position[1],
                                   agent.land_position[2], agent.land_yaw,
                                   'None', 'None', 'None', 'None'])

        d = vertical_distance(agent.position.z, agent.land_position[2])
        if d <= self.distance_to_waypoint_threshold:
            logger.info(agent.name + ' Landing completed')
            agent.stop()

    @staticmethod
    def standby_control_law(agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.position.x, agent.position.y, agent.position.z, agent.yaw,
                                   agent.velocity.x, agent.velocity.y, agent.velocity.z,
                                   agent.standby_position[0], agent.standby_position[1],
                                   agent.standby_position[2], agent.standby_yaw,
                                   'None', 'None', 'None', 'None'])

    @staticmethod
    def circle(agent: Agent):
        try:
            roll, pitch, yaw, thrust, _, __ = uav_control_law.circle(agent)
            agent.circle_t = agent.circle_t + agent.delta_t
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        except Exception as e:
            logger.error('Error "' + str(e) + '" detected in the uav_control_law.circle() function')
            agent.error = e
            agent.standby()

    @staticmethod
    def circle_tangent_x_axis(agent: Agent):
        try:
            roll, pitch, yaw, thrust, _, __ = uav_control_law.circle_tangent_x_axis(agent)
            agent.circle_t = agent.circle_t + agent.delta_t
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        except Exception as e:
            logger.error('Error "' + str(e) + '" detected in the uav_control_law.circle_tangent_x_axis() function')
            agent.error = e
            agent.standby()

    def point_of_interest(self, agent: Agent):
        try:
            roll, pitch, yaw, thrust, _, __ = uav_control_law.point_of_interest(agent, self.robot_list[0])
            agent.circle_t = agent.circle_t + agent.delta_t
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        except Exception as e:
            logger.error('Error "' + str(e) + '" detected in the uav_control_law.point_of_interest() function')
            agent.error = e
            agent.standby()


def distance(position_1_xyz_list: List[float], position_2_xyz_list: List[float]):
    d = numpy.sqrt((position_1_xyz_list[0] - position_2_xyz_list[0]) ** 2
                   + (position_1_xyz_list[1] - position_2_xyz_list[1]) ** 2
                   + (position_1_xyz_list[2] - position_2_xyz_list[2]) ** 2)
    return d


def vertical_distance(position_1_z: float, position_2_z: float):
    vd = numpy.sqrt((position_1_z - position_2_z) ** 2)
    return vd
