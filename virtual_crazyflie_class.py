import numpy
import cflib.crtp
import datetime
import logging

from cflib.crazyflie import Crazyflie
from cflib.utils.callbacks import Caller

from basic_math_tools import Integrator, Differentiator, PID, find_min_angle_difference
from Coordinates_class import CoordinatesAngles6D, AttitudeCommand, VelocityCommand, PositionCommand

logger = logging.getLogger(__name__)


class VirtualCrazyflie(Crazyflie):
    def __init__(self, *args, **kwargs):
        super(VirtualCrazyflie, self).__init__(*args, **kwargs)

        self.battery_level_retrieved = Caller()
        self.initial_battery_level: float = 0

        self.check_attitude_callback = None
        self.check_battery_mode_callback = None

        self.commander = VirtualCommander(self)
        self.position: CoordinatesAngles6D = CoordinatesAngles6D()
        self.velocity: CoordinatesAngles6D = CoordinatesAngles6D()
        self.acceleration: CoordinatesAngles6D = CoordinatesAngles6D()

        self.timestamp: float = 0

        self.yaw_position_calculator: Integrator = Integrator()

        self.x_velocity_calculator: Differentiator = Differentiator()
        self.y_velocity_calculator: Differentiator = Differentiator()
        self.z_velocity_calculator: Differentiator = Differentiator()
        self.roll_velocity_calculator: Differentiator = Differentiator()
        self.pitch_velocity_calculator: Differentiator = Differentiator()

        self.x_acceleration_calculator: Differentiator = Differentiator()
        self.y_acceleration_calculator: Differentiator = Differentiator()
        self.z_acceleration_calculator: Differentiator = Differentiator()
        self.roll_acceleration_calculator: Differentiator = Differentiator()
        self.pitch_acceleration_calculator: Differentiator = Differentiator()
        self.yaw_acceleration_calculator: Differentiator = Differentiator()

        self.roll_acceleration_response = Differentiator()
        self.pitch_acceleration_response = Differentiator()
        self.yaw_acceleration_response = Differentiator()

        self.x_velocity_response = Integrator()
        self.y_velocity_response = Integrator()
        self.z_velocity_response = Integrator()
        self.roll_velocity_response = Differentiator()
        self.pitch_velocity_response = Differentiator()
        self.yaw_velocity_response = Differentiator()

        self.x_position_response = Integrator()
        self.y_position_response = Integrator()
        self.z_position_response = Integrator()

        self.mass: float = 0.030  # kg

    def open_link(self, link_uri):
        """
        Open the communication link to a copter at the given URI and setup the
        connection (download log/parameter TOC).
        """
        print(link_uri + ' open link')
        logger.info('Opening link')
        self.connection_requested.call(link_uri)
        self.link_uri = link_uri
        self.connected_ts = datetime.datetime.now()
        self.connected.call(self.link_uri)
        self.state = cflib.crazyflie.State.INITIALIZED
        self.fully_connected.call(self.link_uri)

    def close_link(self):
        """Close the communication link."""
        logger.info('Closing link')
        self.commander.send_setpoint(AttitudeCommand(0, 0, 0, 0))
        self._answer_patterns = {}
        self.state = cflib.crazyflie.State.DISCONNECTED
        self.disconnected.call(self.link_uri)

    def get_battery_level(self):
        self.initial_battery_level = 100
        self.battery_level_retrieved.call()

    def spawn(self, position: CoordinatesAngles6D):
        self.position = position

        self.x_position_response.reset(self.timestamp, initial_value=self.position.x)
        self.y_position_response.reset(self.timestamp, initial_value=self.position.y)
        self.z_position_response.reset(self.timestamp, initial_value=self.position.z)

        self.x_velocity_response.reset(self.timestamp, initial_value=0)
        self.y_velocity_response.reset(self.timestamp, initial_value=0)
        self.z_velocity_response.reset(self.timestamp, initial_value=0)

        self.yaw_position_calculator.reset(self.timestamp, initial_value=self.position.yaw)

    def get_flight_response(self):
        self.position.roll = self.commander.command.roll
        self.position.pitch = self.commander.command.pitch
        self.velocity.yaw = -self.commander.command.yaw_rate
        thrust = self.commander.command.thrust

        # Yaw position
        self.position.yaw = self.yaw_position_calculator.integrate(self.velocity.yaw, self.timestamp)

        # Units convert from degrees to radians
        roll_in_radians = self.position.roll * numpy.pi / 180
        pitch_in_radians = self.position.pitch * numpy.pi / 180
        yaw_in_radians = self.position.yaw * numpy.pi / 180

        # Acceleration
        ax_n = 9.81 * pitch_in_radians
        ay_n = - 9.81 * roll_in_radians
        ax = (ax_n * numpy.cos(yaw_in_radians)
              - ay_n * numpy.sin(yaw_in_radians))
        ay = (ax_n * numpy.sin(yaw_in_radians)
              + ay_n * numpy.cos(yaw_in_radians))
        az = (thrust / (2 * 65535)) / self.mass - 9.81

        # Velocity
        vx = self.x_velocity_response.integrate(ax, self.timestamp)
        vy = self.y_velocity_response.integrate(ay, self.timestamp)
        vz = self.z_velocity_response.integrate(az, self.timestamp)

        # Position
        self.position.x = self.x_position_response.integrate(vx, self.timestamp)
        self.position.y = self.y_position_response.integrate(vy, self.timestamp)
        self.position.z = self.z_position_response.integrate(vz, self.timestamp)

        if self.position.z < 0:
            self.position.z = 0
            self.z_velocity_response.reset(self.timestamp, 0)
            self.z_position_response.reset(self.timestamp, 0)

        if self.check_attitude_callback:
            self.check_attitude_callback()
        if self.check_battery_mode_callback:
            self.check_battery_mode_callback(0)

    def measure_state(self):
        # Velocity
        self.velocity.x = self.x_velocity_calculator.differentiate(self.position.x, self.timestamp)
        self.velocity.y = self.y_velocity_calculator.differentiate(self.position.y, self.timestamp)
        self.velocity.z = self.z_velocity_calculator.differentiate(self.position.z, self.timestamp)
        self.velocity.roll = self.roll_velocity_calculator.differentiate(self.position.roll, self.timestamp)
        self.velocity.pitch = self.pitch_velocity_calculator.differentiate(self.position.pitch, self.timestamp)

        # Acceleration
        self.acceleration.x = self.x_acceleration_calculator.differentiate(self.velocity.x, self.timestamp)
        self.acceleration.y = self.y_acceleration_calculator.differentiate(self.velocity.y, self.timestamp)
        self.acceleration.z = self.z_acceleration_calculator.differentiate(self.velocity.z, self.timestamp)
        self.acceleration.roll = self.roll_acceleration_calculator.differentiate(self.velocity.roll, self.timestamp)
        self.acceleration.pitch = self.pitch_acceleration_calculator.differentiate(self.velocity.pitch, self.timestamp)
        self.acceleration.yaw = self.yaw_acceleration_calculator.differentiate(self.velocity.yaw, self.timestamp)

        # print(self.position)

    def start_attitude_logs(self):
        pass

    def set_parameters(self):
        pass


class VirtualCommander:
    def __init__(self, crazyflie=None):
        """
        Initialize the commander object. By default, the commander is in
        +-mode (not x-mode).
        """
        self._cf: VirtualCrazyflie = crazyflie
        self._x_mode = False

        self.command: AttitudeCommand = AttitudeCommand()

        self.x_velocity_control: PID = PID(kp=2, ki=0.1, kd=0.01,  # kp 2.5
                                           output_at_steady_state=0,
                                           min_output=-1, max_output=1)

        self.y_velocity_control: PID = PID(kp=2, ki=0.1, kd=0.01,  # kp 2.5
                                           output_at_steady_state=0,
                                           min_output=-1, max_output=1)

        self.z_velocity_control: PID = PID(kp=1, ki=0.1, kd=0.05,
                                           output_at_steady_state=0,
                                           min_output=-2, max_output=2)

        self.roll_position_control: PID = PID(kp=1.2, ki=0.1, kd=0.1,  # kp 1, ki 0.1, kd 0.1
                                              output_at_steady_state=0,
                                              min_output=-20 * numpy.pi / 180, max_output=20 * numpy.pi / 180)

        self.pitch_position_control: PID = PID(kp=1.2, ki=0.1, kd=0.1,
                                               output_at_steady_state=0,
                                               min_output=-20 * numpy.pi / 180, max_output=20 * numpy.pi / 180)

        self.yaw_velocity_control: PID = PID(kp=2.5, ki=0.1, kd=0.2,
                                             output_at_steady_state=0,
                                             min_output=-180, max_output=180)

        self.thrust_control: PID = PID(kp=32500,  # 32500
                                       ki=8125,  # 8125
                                       kd=1000,  # 1000 | 16250
                                       output_at_steady_state=38000,
                                       min_output=0, max_output=65535)

    def reset_pid(self, timestamp):
        self.x_velocity_control.reset(timestamp)
        self.y_velocity_control.reset(timestamp)
        self.z_velocity_control.reset(timestamp)
        self.roll_position_control.reset(timestamp)
        self.pitch_position_control.reset(timestamp)
        self.yaw_velocity_control.reset(timestamp)
        self.thrust_control.reset(timestamp)

    def get_attitude_command(self, velocity_command: VelocityCommand):
        velocity_command.saturate()
        yaw_in_radians = self._cf.position.yaw * numpy.pi / 180

        # Velocity command in the UAV frame
        vx_n_command = (velocity_command.vx * numpy.cos(yaw_in_radians) +
                        velocity_command.vy * numpy.sin(yaw_in_radians))
        vy_n_command = (- velocity_command.vx * numpy.sin(yaw_in_radians) +
                        velocity_command.vy * numpy.cos(yaw_in_radians))

        # Actual velocity in the UAV frame
        vx_n_observed = (self._cf.velocity.x * numpy.cos(yaw_in_radians)
                         + self._cf.velocity.y * numpy.sin(yaw_in_radians))
        vy_n_observed = (- self._cf.velocity.x * numpy.sin(yaw_in_radians)
                         + self._cf.velocity.y * numpy.cos(yaw_in_radians))

        vx_n_error = vx_n_command - vx_n_observed
        vy_n_error = vy_n_command - vy_n_observed
        vz_error = velocity_command.vz - self._cf.velocity.z

        roll_command_in_radians = self.roll_position_control.pid(-vy_n_error, self._cf.timestamp)
        pitch_command_in_radians = self.pitch_position_control.pid(vx_n_error, self._cf.timestamp)
        thrust_command = round(self.thrust_control.pid(vz_error, self._cf.timestamp))

        attitude_command = AttitudeCommand(roll=roll_command_in_radians * 180 / numpy.pi,
                                           pitch=pitch_command_in_radians * 180 / numpy.pi,
                                           yaw_rate=velocity_command.yaw_rate,
                                           thrust=thrust_command)
        return attitude_command

    def get_velocity_command(self, position_command):
        position_command.saturate()
        x_error = position_command.x - self._cf.position.x
        y_error = position_command.y - self._cf.position.y
        z_error = position_command.z - self._cf.position.z
        yaw_error = find_min_angle_difference(position_command.yaw, self._cf.position.yaw)

        vx_command = self.x_velocity_control.pid(x_error, self._cf.timestamp)
        vy_command = self.y_velocity_control.pid(y_error, self._cf.timestamp)
        vz_command = self.z_velocity_control.pid(z_error, self._cf.timestamp)
        yaw_rate_command = self.yaw_velocity_control.pid(-yaw_error, self._cf.timestamp)

        velocity_command = VelocityCommand(vx=vx_command,
                                           vy=vy_command,
                                           vz=vz_command,
                                           yaw_rate=yaw_rate_command)
        return velocity_command

    def set_client_xmode(self, enabled):
        """
        Enable/disable the client side X-mode. When enabled this recalculates
        the setpoints before sending them to the Crazyflie.
        """
        self._x_mode = enabled

    def send_setpoint(self, attitude_command: AttitudeCommand):
        """
        Send a new control setpoint for roll/pitch/yaw_Rate/thrust to the copter.

        The meaning of these values is depended on the mode of the RPYT commander in the firmware
        Default settings are Roll, pitch, yaw_rate and thrust

        roll,  pitch are in degrees
        yaw_rate is in degrees/s
        thrust is an integer value ranging from 10001 (next to no power) to 60000 (full power)
        """
        attitude_command.saturate()
        if attitude_command.thrust > 0xFFFF or attitude_command.thrust < 0:
            raise ValueError('Thrust must be between 0 and 0xFFFF')

        if self._x_mode:
            roll = 0.707 * (attitude_command.roll - attitude_command.pitch)
            pitch = 0.707 * (attitude_command.roll + attitude_command.pitch)
            attitude_command.roll = roll
            attitude_command.pitch = pitch

        self.command = attitude_command

    def send_stop_setpoint(self):
        """
        Send STOP setpoing, stopping the motors and (potentially) falling.
        """
        self.send_setpoint(AttitudeCommand(0, 0, 0, 0))

    def send_velocity_world_setpoint(self, velocity_command: VelocityCommand):
        """
        Send Velocity in the world frame of reference setpoint with yawrate commands

        vx, vy, vz are in m/s
        yawrate is in degrees/s
        """
        attitude_command = self.get_attitude_command(velocity_command=velocity_command)
        self.send_setpoint(attitude_command)

    def send_position_setpoint(self, position_command: PositionCommand):
        """
        Control mode where the position is sent as absolute (world) x,y,z coordinate in
        meter and the yaw is the absolute orientation.

        x, y, z are in m
        yaw is in degrees
        """
        velocity_command = self.get_velocity_command(position_command=position_command)
        self.send_velocity_world_setpoint(velocity_command)
