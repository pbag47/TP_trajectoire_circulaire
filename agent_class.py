import _csv
import cflib.crtp
import time

import numpy as np
# noinspection PyProtectedMember
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from qtm.packet import RT3DMarkerPositionNoLabel
from typing import List, Union


class Agent:
    def __init__(self, name: str, uri: str):
        self.name:  str = name
        self.uri:   str = uri

        # ---- Parameters ---- #
        self.max_roll:  float = 45  # Maximum |roll| tolerance before emergency stop (°)
        self.max_pitch: float = 45  # Maximum |pitch| tolerance before emergency stop (°)

        self.x_boundaries: List[float] = [-1.5, 1.5]  # [Minimum x coordinate (m), Maximum x coordinate (m)]
        self.y_boundaries: List[float] = [-1.5, 1.5]  # [Minimum y coordinate (m), Maximum y coordinate (m)]
        self.z_boundaries: List[float] = [-0.1, 1.50]  # [Minimum z coordinate (m), Maximum z coordinate (m)]

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
        self.cf:                    Crazyflie = Crazyflie()
        self.state:                 str = 'Not flying'
        self.is_flying:             bool = False
        self.enabled:               bool = False
        self.battery_test_passed:   bool = False
        self.extpos_test_passed:    bool = False

        self.setup_finished:            bool = False
        self.initial_battery_voltage:   Union[None, float] = None   # (V)
        self.initial_battery_level:     Union[None, int] = None     # (%)

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
        self.velocity:              [float] * 3 = [0.0, 0.0, 0.0]   # ([m/s, m/s, m/s])
        self.delta_t:               float = 1.0                     # (s)
        self.circle_t:              float = 0.0                     # (s)
        self.previous_iz:           float = 0.0
        self.invalid_6dof_count:    int = 0

        self.extpos: RT3DMarkerPositionNoLabel = RT3DMarkerPositionNoLabel(0, 0, 0, None)
        self.csv_logger: Union[None, _csv.writer] = None

    def connect_cf(self):
        self.cf.open_link(self.uri)
        self.cf.connected.add_callback(self.cf_connected_callback)
        self.cf.disconnected.add_callback(self.cf_disconnected_callback)

    def cf_connected_callback(self, _):
        self.enabled = True
        # self.setup_parameters()
        print('<', self.name, '> : UAV connected')
        self.cf.commander.send_setpoint(0, 0, 0, 0)
        # self.start_attitude_logs()

    def setup_parameters(self):
        # time.sleep(1.5)
        log_config = LogConfig(name=self.name + ' battery voltage', period_in_ms=50)
        log_config.add_variable('pm.vbat', 'float')  # Retrieves the battery level
        self.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(self.check_battery_level)
        log_config.start()
        self.cf.param.set_value('stabilizer.estimator', str(self.stabilizer_estimator))
        self.cf.param.set_value('flightmode.posSet', str(self.flightmode_pos_set))
        self.check_initial_extpos()
        if not self.enabled:
            self.stop()

    def check_battery_level(self, _, data, logconf):
        self.initial_battery_voltage = data['pm.vbat']
        logconf.stop()
        voltage_points = [3.0, 3.6, 3.65, 3.75, 3.85, 4.0, 4.2]
        battery_level_points = [0, 5, 10, 25, 50, 75, 100]
        self.initial_battery_level = np.interp(self.initial_battery_voltage, voltage_points, battery_level_points)
        if self.initial_battery_level <= 20:
            print('---- Warning ---- <', self.name, '> : Low battery level (', round(self.initial_battery_level),
                  '% ), flight disabled')
            self.enabled = False
        else:
            self.battery_test_passed = True

    def check_initial_extpos(self):
        if self.extpos.id is None:
            print('---- Warning ---- <', self.name, '> : Crazyflie not tracked by QTM, flight disabled')
            self.enabled = False
        else:
            self.extpos_test_passed = True

    def start_attitude_logs(self):
        self.cf.log.add_config(self.log_config)
        self.log_config.data_received_cb.add_callback(self.check_attitude)
        self.log_config.start()

    def check_attitude(self, _, data, __):
        if self.enabled and (abs(data['stabilizer.roll']) > self.max_roll
                             or abs(data['stabilizer.pitch']) > self.max_pitch
                             or self.extpos.x < self.x_boundaries[0] or self.extpos.x > self.x_boundaries[1]
                             or self.extpos.y < self.y_boundaries[0] or self.extpos.y > self.y_boundaries[1]
                             or self.extpos.z < self.z_boundaries[0] or self.extpos.z > self.z_boundaries[1]):
            print('---- Warning ---- <', self.name, '> : Abnormal attitude detected')
            print('<', self.name, '> attitude : {Roll =', round(data['stabilizer.roll']),
                  '° ; Pitch =', round(data['stabilizer.pitch']),
                  '° ; x =', round(self.extpos.x, 2), 'm ; y =', round(self.extpos.y, 2), 'm ; z =',
                  round(self.extpos.z, 2), 'm}')
            print('<', self.name, '> nominal attitude boundaries : {|Roll| <', self.max_roll, '° ; |Pitch| <',
                  self.max_pitch,
                  '°, x in', self.x_boundaries, 'm, y in', self.y_boundaries, 'm, z in', self.z_boundaries, 'm}')
            self.stop()
        else:
            self.yaw = data['stabilizer.yaw']
            # print(round(data['stabilizer.thrust']))

        if self.is_flying and round(data['pm.state']) == 3 and self.state != 'Land':
            print('---- Warning ---- <', self.name, '> : Low battery level, automatic landing triggered')
            self.land()

    def cf_disconnected_callback(self, _):
        self.enabled = False
        self.is_flying = False
        self.state = 'Not flying'
        print('<', self.name, '> : UAV disconnected')

    def stop(self):
        if self.setup_finished:
            try:
                self.log_config.stop()
            except AttributeError:
                print('<', self.name, '> Warning : Unable to stop attitude logging')
        self.state = 'Not flying'
        self.is_flying = False
        self.enabled = False
        self.cf.commander.send_stop_setpoint()

    def send_external_position(self):
        self.cf.extpos.send_extpos(self.extpos.x, self.extpos.y, self.extpos.z)

    def update_extpos(self, marker: RT3DMarkerPositionNoLabel, timestamp: float):
        if timestamp > self.timestamp:
            self.delta_t = timestamp - self.timestamp
            self.update_velocity(marker, timestamp)
            self.timestamp = timestamp
            self.extpos = RT3DMarkerPositionNoLabel(marker.x * 10 ** -3, marker.y * 10 ** -3, marker.z * 10 ** -3,
                                                    marker.id)
        else:
            print(' ---- Warning ---- <', self.name, '> : Several extpos packets received for the same timestamp')
            self.stop()

    def update_velocity(self, marker: RT3DMarkerPositionNoLabel, timestamp: float):
        self.velocity[0] = (marker.x * 10 ** -3 - self.extpos.x) / (timestamp - self.timestamp)
        self.velocity[1] = (marker.y * 10 ** -3 - self.extpos.y) / (timestamp - self.timestamp)
        self.velocity[2] = (marker.z * 10 ** -3 - self.extpos.z) / (timestamp - self.timestamp)

    def set_initial_position(self, position: [float] * 3):
        self.initial_position = position

    def set_takeoff_height(self, height: float):
        self.takeoff_height = height

    def standby(self):
        print('<', self.name, '> : Standby')
        self.state = 'Standby'
        self.standby_position = [self.extpos.x, self.extpos.y, self.extpos.z]
        self.standby_yaw = self.yaw

    def takeoff(self):
        print('<', self.name, '> : Takeoff')
        self.state = 'Takeoff'
        self.is_flying = True
        self.takeoff_position = [self.extpos.x, self.extpos.y, self.takeoff_height]
        self.takeoff_yaw = self.yaw

    def land(self):
        print('<', self.name, '> : Landing')
        self.state = 'Land'
        self.land_position = [self.extpos.x, self.extpos.y, 0.0]
        self.land_yaw = self.yaw

    def manual_flight(self):
        print('<', self.name, '> : Manual flight')
        self.state = 'Manual'
        self.standby_position = [self.extpos.x, self.extpos.y, self.extpos.z]
        self.standby_yaw = self.yaw

    def circle(self):
        print('<', self.name, '> : Circle with constant yaw')
        self.circle_t = 0.0
        self.state = 'Circle'

    def circle_with_tangent_x_axis(self):
        print('<', self.name, '> : Circle with x axis tangent to trajectory')
        self.circle_t = 0.0
        self.state = 'Circle with tangent x axis'

    def point_of_interest(self):
        print('<', self.name, '> : Point of Interest')
        self.circle_t = 0.0
        self.state = 'Point of interest'


def test_cf_connection():
    cf_name = 'cf1'
    cf_radio_id = 'radio://0/83/2M/E7E7E7E701'

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
