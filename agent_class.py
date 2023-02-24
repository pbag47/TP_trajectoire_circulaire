import _csv
import cflib.crtp
import logging
import numpy
import time

# noinspection PyProtectedMember
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from qtm.packet import RT3DMarkerPositionNoLabel
from typing import List, Union


logger = logging.getLogger(__name__)


class Agent:
    def __init__(self, name: str, uri: str):
        self.name:  str = name
        self.uri:   str = uri

        # ---- Parameters ---- #
        self.max_roll:  float = 45  # Maximum |roll| tolerance before emergency stop (°)
        self.max_pitch: float = 45  # Maximum |pitch| tolerance before emergency stop (°)

        self.x_boundaries: List[float] = [-1.25, 1.25]  # [Minimum x coordinate (m), Maximum x coordinate (m)]
        self.y_boundaries: List[float] = [-1.25, 1.25]  # [Minimum y coordinate (m), Maximum y coordinate (m)]
        self.z_boundaries: List[float] = [-0.1, 1.20]  # [Minimum z coordinate (m), Maximum z coordinate (m)]

        # Crazyflie parameters:
        # -- Warning -- May cause chaotic flight when badly configured, do not attempt to change these settings without
        # a consistent knowledge of the Crazyflie parameters documentation.
        self.stabilizer_estimator:  int = 2  # 2: stabilizer based on an Extended Kalman Filter
        self.flightmode_pos_set:    int = 0

        # Crazyflie variables log:
        # -- Warning -- The program must at least retrieve the attitude information (roll, pitch and yaw) and the
        # battery mode for each Crazyflie to perform attitude and battery security checks and to control their yaw.
        # The user may add other variables to log (e.g. battery level, radio link quality, etc...) with respects to
        # the radio communication speed limits.
        # Please check the validity of a custom log configuration with the cfclient before implementing it on this
        # program.
        self.log_config = LogConfig(name=self.name + ' attitude', period_in_ms=50)
        self.log_config.add_variable('stabilizer.roll', 'float')  # Retrieves the roll from Crazyflie stabilizer (EKF)
        self.log_config.add_variable('stabilizer.pitch', 'float')  # Retrieves the pitch from Crazyflie stabilizer (EKF)
        self.log_config.add_variable('stabilizer.yaw', 'float')  # Retrieves the yaw from Crazyflie stabilizer (EKF)
        # self.log_config.add_variable('stabilizer.thrust', 'float')
        # Retrieves the battery mode (0 = nominal mode, 3 = Low energy mode)
        self.log_config.add_variable('pm.state', 'float')

        # ---- Attributes initialization ---- #
        self.cf:                    Crazyflie = Crazyflie(rw_cache='./cache')
        self.state:                 str = 'Not flying'
        self.is_flying:             bool = False
        self.enabled:               bool = False
        self.battery_test_passed:   bool = False
        self.position_test_passed:  bool = False

        self.setup_finished:            bool = False
        self.initial_battery_voltage:   float = 0.0                 # (V)
        self.initial_battery_level:     int = 0                     # (%)

        self.initial_position:      [float] * 3 = [0, 0, 0]         # ([m, m, m])

        self.takeoff_position:      List[float] = []                # ([m, m, m])
        self.takeoff_yaw:           float = 0.0                     # (°)
        self.takeoff_height:        float = 0.5                     # (m)

        self.land_position:         List[float] = []                # ([m, m, m])
        self.land_yaw:              float = 0.0                     # (°)

        self.standby_position:      List[float] = []                # ([m, m, m])
        self.standby_yaw:           float = 0.0                     # (°)

        self.timestamp:             float = 0.0                     # (s)

        self.yaw:                   float = 0.0                     # (°)
        self.delta_t:               float = 1.0                     # (s)
        self.circle_t:              float = 0.0                     # (s)
        self.previous_iz:           float = 0.0
        self.invalid_6dof_count:    int = 0

        self.position: RT3DMarkerPositionNoLabel = RT3DMarkerPositionNoLabel(x=0,  # (m)
                                                                             y=0,  # (m)
                                                                             z=0,  # (m)
                                                                             id=None,  # (positive integer, no unit)
                                                                             )
        self.velocity: RT3DMarkerPositionNoLabel = RT3DMarkerPositionNoLabel(x=0,  # (m/s)
                                                                             y=0,  # (m/s)
                                                                             z=0,  # (m/s)
                                                                             id=None,  # (positive integer, no unit)
                                                                             )

        self.csv_logger: Union[None, _csv.writer] = None
        self.error = None

    def connect_cf(self):
        self.cf.open_link(self.uri)
        self.cf.connected.add_callback(self.cf_connected_callback)
        self.cf.fully_connected.add_callback(self.setup_parameters)
        self.cf.connection_failed.add_callback(self.cf_connection_failed_callback)
        self.cf.disconnected.add_callback(self.cf_disconnected_callback)

    def cf_connected_callback(self, _):
        self.enabled = True
        logger.info(self.name + ' connected')
        self.cf.commander.send_setpoint(0, 0, 0, 0)

    def cf_connection_failed_callback(self, _, __):
        self.battery_test_passed = True
        self.position_test_passed = True
        self.enabled = False
        logger.warning(self.name + ' connection attempt failed')

    def setup_parameters(self, _):
        logging.getLogger('__main__').info(self.name + ' connected, starting pre-flight checks and parameters setup')
        logger.info(self.name + ' pre-flight checks and parameters setup')
        log_config = LogConfig(name=self.name + ' battery voltage', period_in_ms=50)
        log_config.add_variable('pm.vbat', 'float')  # Retrieves the battery level
        self.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(self.check_battery_level)
        log_config.start()
        self.cf.param.set_value('stabilizer.estimator', str(self.stabilizer_estimator))
        self.cf.param.set_value('flightmode.posSet', str(self.flightmode_pos_set))
        self.check_initial_position()
        if self.enabled:
            self.start_attitude_logs()
        else:
            self.stop()

    def check_battery_level(self, _, data, logconf):
        self.initial_battery_voltage = data['pm.vbat']
        logconf.stop()
        # Interpolates the received voltage to display it as a battery level percentage
        voltage_points = [
            3.0, 3.6, 3.65, 3.75, 3.85, 4.0, 4.2,
        ]
        battery_level_points = [
            0, 5, 10, 25, 50, 75, 100,
        ]
        self.initial_battery_level = numpy.interp(self.initial_battery_voltage, voltage_points, battery_level_points)
        if self.initial_battery_level <= 20:
            logger.error(self.name + ' low battery level (' + str(round(self.initial_battery_level)) + '%)')
            self.enabled = False
        else:
            logger.info(self.name + ' battery test passed')
            self.battery_test_passed = True

    def check_initial_position(self):
        if self.position.id is None:
            logger.error(self.name + ' not tracked by QTM, flight disabled')
            self.enabled = False
        else:
            logger.info(self.name + ' is tracked by QTM')
            self.position_test_passed = True

    def start_attitude_logs(self):
        self.cf.log.add_config(self.log_config)
        self.log_config.data_received_cb.add_callback(self.check_attitude)
        self.log_config.start()
        logger.info(self.name + ' attitude logging started')

    def check_attitude(self, _, data, __):
        if self.enabled:
            self.yaw = data['stabilizer.yaw']

            if abs(data['stabilizer.roll']) > self.max_roll:
                logger.error(self.name + ' unsafe roll detected : ' + str(round(data['stabilizer.roll'])) + ' °')
                self.stop()
            if abs(data['stabilizer.pitch']) > self.max_pitch:
                logger.error(self.name + ' unsafe pitch detected : ' + str(round(data['stabilizer.pitch'])) + ' °')
                self.stop()
            if self.position.x < self.x_boundaries[0] or self.position.x > self.x_boundaries[1]:
                logger.error(self.name + ' outside boundaries on the x axis : ' + str(round(self.position.x, 2)) + ' m')
                self.stop()
            if self.position.y < self.y_boundaries[0] or self.position.y > self.y_boundaries[1]:
                logger.error(self.name + ' outside boundaries on the y axis : ' + str(round(self.position.y, 2)) + ' m')
                self.stop()
            if self.position.z < self.z_boundaries[0] or self.position.z > self.z_boundaries[1]:
                logger.error(self.name + ' outside boundaries on the z axis : ' + str(round(self.position.z, 2)) + ' m')
                self.stop()
            if round(data['pm.state']) == 3 and self.state != 'Land':
                logger.error(self.name + ' low battery level, automatic landing triggered')
                self.land()

    def cf_disconnected_callback(self, _):
        self.enabled = False
        self.is_flying = False
        self.state = 'Not flying'
        logger.warning(self.name + ' disconnected')

    def stop(self):
        if self.setup_finished:
            try:
                self.log_config.stop()
            except AttributeError:
                logger.info(self.name + ' unable to stop attitude logging')
        self.state = 'Not flying'
        self.is_flying = False
        self.enabled = False
        self.cf.commander.send_stop_setpoint()
        logger.info(self.name + ' stopped')

    def send_external_position(self):
        self.cf.extpos.send_extpos(self.position.x, self.position.y, self.position.z)

    def update_position(self, marker: RT3DMarkerPositionNoLabel, timestamp: float):
        if timestamp > self.timestamp:
            self.delta_t = timestamp - self.timestamp
            self.update_velocity(marker)
            self.timestamp = timestamp
            self.position = RT3DMarkerPositionNoLabel(marker.x * 10 ** -3, marker.y * 10 ** -3, marker.z * 10 ** -3,
                                                      marker.id)
        else:
            logger.error(self.name + ' several QTM packets received for the same timestamp')
            self.stop()

    def update_velocity(self, marker: RT3DMarkerPositionNoLabel):
        self.velocity = RT3DMarkerPositionNoLabel(
            x=(marker.x * 10 ** -3 - self.position.x) / self.delta_t,
            y=(marker.y * 10 ** -3 - self.position.y) / self.delta_t,
            z=(marker.z * 10 ** -3 - self.position.z) / self.delta_t,
            id=marker.id,
        )

    def set_initial_position(self, position: [float] * 3):
        self.initial_position = position
        logger.debug(self.name + ' initial position set')

    def set_takeoff_height(self, height: float):
        self.takeoff_height = height
        logger.debug(self.name + ' takeoff height set')

    def standby(self):
        logger.info(self.name + ' Switch to Standby mode')
        self.state = 'Standby'
        self.standby_position = [self.position.x, self.position.y, self.position.z]
        self.standby_yaw = self.yaw

    def takeoff(self):
        logger.info(self.name + ' Switch to Takeoff mode')
        self.state = 'Takeoff'
        self.is_flying = True
        self.takeoff_position = [self.position.x, self.position.y, self.takeoff_height]
        self.takeoff_yaw = self.yaw

    def land(self):
        logger.info(self.name + ' Switch to Landing mode')
        self.state = 'Land'
        self.land_position = [self.position.x, self.position.y, 0.0]
        self.land_yaw = self.yaw

    def manual_flight(self):
        logger.info(self.name + ' Switch to Manual flight mode')
        self.state = 'Manual'
        self.standby_position = [self.position.x, self.position.y, self.position.z]
        self.standby_yaw = self.yaw

    def circle(self):
        logger.info(self.name + ' Circle with constant yaw')
        self.circle_t = 0.0
        self.state = 'Circle'

    def circle_with_tangent_x_axis(self):
        logger.info(self.name + ' Circle with x axis tangent to trajectory')
        self.circle_t = 0.0
        self.state = 'Circle with tangent x axis'

    def point_of_interest(self):
        logger.info(self.name + ' Point of Interest')
        self.circle_t = 0.0
        self.state = 'Point of interest'


def test_cf_connection():
    cf_name = 'cf1'
    cf_radio_id = 'radio://0/90/2M/E7E7E7E701'

    agt = Agent(cf_name, cf_radio_id)
    cflib.crtp.init_drivers()
    agt.connect_cf()
    time.sleep(10)
    agt.stop()
    agt.cf.close_link()


if __name__ == '__main__':
    """
    Test de connexion par radio à un drone Crazyflie.
    Dans la fonction 'test_cf_connection()', renseignez le nom du drone que vous voulez connecter (choix arbitraire),
    ainsi que son identifiant radio.
    
    Comportement attendu :
    Une fois le drone mis sous tension, l'exécution du programme de test aboutit à la connection au drone
    (sans faire tourner ses moteurs), puis un avertissement lié à l'absence de mesure de position du drone par QTM
    apparaît, et le drone est déconnecté au bout de 10s.
    """
    test_cf_connection()
