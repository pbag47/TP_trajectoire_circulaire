from joystick_map import joystick_map
from pyjoystick.sdl2 import Key, Joystick, JoystickEventLoop
from swarm_object_class import SwarmObject
from threading import Thread
from typing import Union


class UserInputManager:
    def __init__(self, swarm_object: Union[SwarmObject, None] = None):
        self.swarm = swarm_object
        self.button_names, self.axis_names = joystick_map('Microsoft X-Box One S pad')
        self.joystick_event_loop = JoystickEventLoop(self.device_added, self.device_removed, self.key_received)
        self.thread = Thread(target=self.start_ui_thread)
        self.thread.start()

    def start_ui_thread(self):
        self.joystick_event_loop.run()

    @staticmethod
    def device_added(joystick: Joystick):
        print('Gamepad "', joystick.name,
              '" detected with', joystick.get_numaxes(), 'axes and',
              joystick.get_numbuttons(), 'buttons')

    @staticmethod
    def device_removed(joystick):
        print(joystick, 'removed')

    def key_received(self, key: Key):
        if key.keytype == Key.AXIS:
            axis = self.axis_names[key.number]
            if axis == 'pitch':
                if -0.01 < key.value < 0.01:
                    self.swarm.manual_x = 0.0
                else:
                    self.swarm.manual_x = key.value ** 3

            if axis == 'roll':
                if -0.01 < key.value < 0.01:
                    self.swarm.manual_y = 0.0
                else:
                    self.swarm.manual_y = key.value ** 3

            if axis == 'yaw':
                self.swarm.manual_yaw = self.swarm.manual_yaw - 2 * key.value ** 3

            if axis == 'yaw-':
                fvalue = (1 + key.value) / 2
                self.swarm.manual_yaw = self.swarm.manual_yaw + 2 * fvalue ** 3

            if axis == 'yaw+':
                fvalue = (1 + key.value) / 2
                self.swarm.manual_yaw = self.swarm.manual_yaw - 2 * fvalue ** 3

            if axis == 'height':
                self.swarm.manual_z = (1 - key.value) / 2

            if axis == 'height2':
                self.swarm.manual_z = self.swarm.manual_z - 0.01 * key.value

        if key.keytype == Key.BUTTON and key.value:
            button = self.button_names[key.number]
            if button == 'Stop':
                print('Stop button triggered, joystick disconnected')
                if self.swarm:
                    for agt in self.swarm.swarm_agent_list:
                        agt.cf.commander.send_stop_setpoint()
                        agt.stop()
                self.joystick_event_loop.stop()

            if button == 'Takeoff / Land':
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and not agt.is_flying:
                        agt.takeoff()
                    elif agt.enabled and agt.is_flying:
                        agt.land()

            if button == 'Standby':
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying:
                        agt.standby()

            if button == 'Manual flight':
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying and any([agt.name == manual for manual in
                                                              self.swarm.manual_flight_agents_list]):
                        self.swarm.manual_z = agt.extpos.z
                        agt.manual_flight()

            if button == 'Circle':
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying:
                        agt.circle()

            if button == 'Circle with tangent x axis':
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying:
                        agt.circle_with_tangent_x_axis()

            if button == 'Point of interest':
                for agt in self.swarm.swarm_agent_list:
                    if agt.enabled and agt.is_flying:
                        agt.point_of_interest()

            if button == 'Yaw-':
                self.swarm.manual_yaw = self.swarm.manual_yaw + 22.5

            if button == 'Yaw+':
                self.swarm.manual_yaw = self.swarm.manual_yaw - 22.5


def test_joystick_input():
    swarm = SwarmObject()
    _ = UserInputManager(swarm)


if __name__ == '__main__':
    """
    Test de lecture des informations provenant du Joystick.

    Comportement attendu :
    Une fois le Joystick branché, l'exécution du programme lit les entrées du Joystick et les affiche dans la console
    jusqu'à son interruption forcée (ctrl+F2)
    """
    test_joystick_input()
