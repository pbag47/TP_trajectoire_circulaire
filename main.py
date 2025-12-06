import asyncio
import cf_info
import cflib.crtp
import csv
import logging
import pynput.keyboard
import qtm_tools
import os
import sys

from PySide6 import QtWidgets
from quamash import QSelectorEventLoop
from qtm_rt import QRTConnection

from joystick_class import Joystick
from robot_class import Robot
from swarm_object_class import SwarmObject
from UI import SetupUI


logger = logging.getLogger(__name__)


async def keyboard_handler():
    global SWARM_MANAGER

    key_queue = detect_keyboard_input()
    while True:
        key = await key_queue.get()
        if key == pynput.keyboard.Key.esc:
            logger.info('Esc key pressed, disconnecting')
            for agt in SWARM_MANAGER.swarm_agent_list:
                agt.cf.commander.send_stop_setpoint()
                agt.stop()
            asyncio.get_event_loop().stop()


def detect_keyboard_input():
    queue = asyncio.Queue()
    loop = asyncio.get_event_loop()

    def on_press_callback(key):
        try:
            loop.call_soon_threadsafe(queue.put_nowait, key.char)
        except AttributeError:
            loop.call_soon_threadsafe(queue.put_nowait, key)

    pynput.keyboard.Listener(on_press=on_press_callback).start()
    return queue



def main():
    # -- Flight parameters ------------------------------------------------------- #
    qtm_ip_address: str = '192.168.0.1'
    agents_list = cf_info.init_agents()
    rbt = Robot('Cible')

    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s - %(levelname)s -  %(name)s  - %(message)s',
        filename=__name__ + '.log',
    )

    settings_app = QtWidgets.QApplication([])
    parameters_file_name = os.path.join('..', 'flight_parameters.txt')
    setup_ui = SetupUI(parameters_filename=parameters_file_name)
    setup_ui.show()
    # -- Settings app         ---------------------------------------------------- #
    exit_code = settings_app.exec()
    # -- Settings app         ---------------------------------------------------- #
    setup_ui.stop()

    for vehicle in setup_ui.vehicle_markers:




    logger.info('UI real-time processing loop started')
    asyncio.ensure_future(start_qtm_streaming(qtm_connection, user_window.update_graph))
    exit_code = settings_app.exec()
    logger.info('UI real-time processing loop stopped')
    asyncio.get_event_loop().run_until_complete(stop_qtm_streaming(qtm_connection))

    # -- Vehicle objects retrieval --------------------------------------------------- #
    uav = user_window.uav
    target = user_window.robot

    # -- Logs initialization ----------------------------------------------------- #
    file = open('logs.csv', 'w')
    writer = csv.writer(file)
    writer.writerow(['Crazyflie name', 'QTM packet timestamp (s)',
                     'QTM_x (m)', 'QTM_y (m)', 'QTM_z (m)', 'cf_yaw (°)',
                     'QTM_vx (m/s)', 'QTM_vy (m/s)', 'QTM_vz (m/s)',
                     'x_g (m)', 'y_g (m)', 'z_g (m)', 'yaw_g (°)',
                     'roll_c (°)', 'pitch_c (°)', 'yaw_rate_c (°/s)', 'thrust_c (PWM)'])

    # -- Flight parameters ------------------------------------------------------- #
    uav.csv_logger = writer
    if target:
        target.csv_logger = writer

    # -- Initial frame acquisition ---------------------------- #
    header, markers, timestamp = qtm_tools.frame_acquisition(qtm_connection)
    logger.info(str(header.marker_count) + ' markers found by QTM during initialization')
    qtm_tools.initial_detection(uav, target, markers, timestamp)

    # -- Crazyflie connection and swarm initialization procedure ----------------- #
    cflib.crtp.init_drivers()
    SWARM_MANAGER = SwarmObject()
    _ = Joystick(SWARM_MANAGER)

    uav.connect_cf()
    SWARM_MANAGER.add_agent(uav)
    SWARM_MANAGER.manual_flight_agents_list.append(uav.name)
    if target:
        SWARM_MANAGER.robot_list = [target]

    # -- Real-time stream start -------------------------------------------------- #
    RUN_TRACKER = True
    asyncio.ensure_future(start_qtm_streaming(qtm_connection, packet_reception_callback))
    asyncio.ensure_future(keyboard_handler())
    logger.info('Real-time processing loop started')
    asyncio.get_event_loop().run_forever()
    logger.info('Real-time processing loop stopped')

    # -- Disconnects the Crazyflies, stops the QTM stream and disconnects QTM ---- #
    uav.stop()
    uav.cf.close_link()

    asyncio.get_event_loop().run_until_complete(qtm_tools.disconnect_qtm(qtm_connection))
    file.close()

    if uav.error:
        raise uav.error

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
