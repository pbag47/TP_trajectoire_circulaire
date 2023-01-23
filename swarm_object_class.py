import numpy
import uav_control_law

from agent_class import Agent
from robot_class import Robot
from typing import List


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
                if agent.enabled and agent.state != 'Not flying':
                    if agent.state == 'Standby':
                        self.standby_control_law(agent)

                    if agent.state == 'Takeoff':
                        self.takeoff_control_law(agent)

                    if agent.state == 'Land':
                        self.landing_control_law(agent)

                    if agent.state == 'Manual':
                        self.manual_control_law(agent)

                    if agent.state == 'Circle':
                        self.circle(agent)

                    if agent.state == 'Circle with tangent x axis':
                        self.circle_tangent_x_axis(agent)

                    if agent.state == 'Point of interest':
                        self.point_of_interest(agent)
                else:
                    agent.cf.commander.send_stop_setpoint()
        else:
            if all([(agent.battery_test_passed and agent.extpos_test_passed) for agent in self.swarm_agent_list]):
                self.ready_to_fly = True
                print('UAV connection recap :')
                for agent in self.swarm_agent_list:
                    if distance([agent.extpos.x, agent.extpos.y, agent.extpos.z], agent.initial_position) > 0.5:
                        agent.stop()
                    agent.setup_finished = True
                    print('    - <', agent.name, '> :')
                    if agent.extpos.x >= 0:
                        print('        x = ', round(agent.extpos.x, 3), 'm')
                    else:
                        print('        x =', round(agent.extpos.x, 3), 'm')
                    if agent.extpos.y >= 0:
                        print('        y = ', round(agent.extpos.y, 3), 'm')
                    else:
                        print('        y =', round(agent.extpos.y, 3), 'm')
                    if agent.extpos.z >= 0:
                        print('        z = ', round(agent.extpos.z, 3), 'm')
                    else:
                        print('        z =', round(agent.extpos.z, 3), 'm')
                    print('        Battery level =', round(agent.initial_battery_level), '%')
                    if agent.enabled:
                        print('        Flight enabled')
                    else:
                        print('        -- Warning -- : Flight disabled')

    def manual_control_law(self, agent: Agent):
        kp = 0.50
        ks = 0.1
        if self.manual_x >= 0:
            agent.standby_position[0] = agent.extpos.x - kp * numpy.sqrt(self.manual_x)
            if agent.standby_position[0] < agent.x_boundaries[0] + ks:
                agent.standby_position[0] = agent.x_boundaries[0] + ks
        else:
            agent.standby_position[0] = agent.extpos.x + kp * numpy.sqrt(-self.manual_x)
            if agent.standby_position[0] > agent.x_boundaries[1] - ks:
                agent.standby_position[0] = agent.x_boundaries[1] - ks
        if self.manual_y >= 0:
            agent.standby_position[1] = agent.extpos.y - kp * numpy.sqrt(self.manual_y)
            if agent.standby_position[1] < agent.y_boundaries[0] + ks:
                agent.standby_position[1] = agent.y_boundaries[0] + ks
        else:
            agent.standby_position[1] = agent.extpos.y + kp * numpy.sqrt(-self.manual_y)
            if agent.standby_position[1] > agent.y_boundaries[1] - ks:
                agent.standby_position[1] = agent.y_boundaries[1] - ks
        agent.standby_position[2] = self.manual_z
        agent.standby_yaw = self.manual_yaw

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   agent.standby_position[0], agent.standby_position[1],
                                   agent.standby_position[2], agent.standby_yaw,
                                   'None', 'None', 'None', 'None'])

        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

    def takeoff_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.takeoff_position[0], agent.takeoff_position[1],
                                                  agent.takeoff_position[2], agent.takeoff_yaw)

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   agent.takeoff_position[0], agent.takeoff_position[1],
                                   agent.takeoff_position[2], agent.takeoff_yaw,
                                   'None', 'None', 'None', 'None'])

        d = distance([agent.extpos.x, agent.extpos.y,
                      agent.extpos.z], agent.takeoff_position)
        if d <= self.distance_to_waypoint_threshold:
            print('<', agent.name, '> : Takeoff completed')
            agent.is_flying = True
            agent.standby()
            agent.standby_position[2] = agent.takeoff_height

    def landing_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.land_position[0], agent.land_position[1],
                                                  agent.land_position[2], agent.land_yaw)

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
                                   agent.land_position[0], agent.land_position[1],
                                   agent.land_position[2], agent.land_yaw,
                                   'None', 'None', 'None', 'None'])

        d = vertical_distance(agent.extpos.z, agent.land_position[2])
        if d <= self.distance_to_waypoint_threshold:
            print('<', agent.name, '> : Landing completed')
            agent.stop()

    @staticmethod
    def standby_control_law(agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

        agent.csv_logger.writerow([agent.name, agent.timestamp,
                                   agent.extpos.x, agent.extpos.y, agent.extpos.z, agent.yaw,
                                   agent.velocity[0], agent.velocity[1], agent.velocity[2],
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
            print(' -- Warning -- Error |', e, '| detected in uav_control_law.circle() function,'
                                               ' switching to Standby state')
            agent.standby()

    @staticmethod
    def circle_tangent_x_axis(agent: Agent):
        try:
            roll, pitch, yaw, thrust, _, __ = uav_control_law.circle_tangent_x_axis(agent)
            agent.circle_t = agent.circle_t + agent.delta_t
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        except Exception as e:
            print(' -- Warning -- Error |', e, '| detected in uav_control_law.circle_tangent_x_axis() function,'
                                               ' switching to Standby state')
            agent.standby()

    def point_of_interest(self, agent: Agent):
        try:
            roll, pitch, yaw, thrust, _, __ = uav_control_law.point_of_interest(agent, self.robot_list[0])
            agent.circle_t = agent.circle_t + agent.delta_t
            agent.cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        except Exception as e:
            print(' -- Warning -- Error |', e, '| detected in uav_control_law.point_of_interest() function,'
                                               ' switching to Standby state')
            agent.standby()


def distance(position_1_xyz_list: List[float], position_2_xyz_list: List[float]):
    d = numpy.sqrt((position_1_xyz_list[0] - position_2_xyz_list[0]) ** 2
                   + (position_1_xyz_list[1] - position_2_xyz_list[1]) ** 2
                   + (position_1_xyz_list[2] - position_2_xyz_list[2]) ** 2)
    return d


def vertical_distance(position_1_z: float, position_2_z: float):
    vd = numpy.sqrt((position_1_z - position_2_z) ** 2)
    return vd
