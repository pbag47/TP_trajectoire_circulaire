import asyncio
import cflib.crtp
import csv
import pynput.keyboard
import qtm_tools

from agent_class import Agent
from joystick_class import Joystick
from qtm import QRTConnection
from qtm.packet import QRTPacket
from robot_class import Robot
from swarm_object_class import SwarmObject


async def keyboard_handler():
    global SWARM_MANAGER

    key_queue = detect_keyboard_input()
    while True:
        key = await key_queue.get()
        if key == pynput.keyboard.Key.esc:
            print('Disconnecting...')
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


async def start_qtm_streaming(connection: QRTConnection):
    """ Starts a QTM stream, and assigns a callback method to run each time a QRTPacket is received from QTM
     This method is made to run forever in an asyncio event loop """
    print('QTM streaming started')
    await connection.stream_frames(components=['3dnolabels'], on_packet=packet_reception_callback)


def packet_reception_callback(packet: QRTPacket):
    global SWARM_MANAGER
    global RUN_TRACKER

    if not RUN_TRACKER:
        print(' ---- Warning ---- Callback execution interrupted by new QTM packet')
        print('                   -> There might be an error occurring during callback execution')
        print('                   -> Or, the sequence might require too much computing load')
        for agents_to_stop in SWARM_MANAGER.swarm_agent_list:
            agents_to_stop.stop()
    RUN_TRACKER = False
    timestamp = packet.timestamp * 10**-6
    headers, markers = packet.get_3d_markers_no_label()
    qtm_tools.uav_tracking(SWARM_MANAGER.swarm_agent_list, SWARM_MANAGER.robot_list, markers, timestamp)
    SWARM_MANAGER.flight_sequence()
    RUN_TRACKER = True


def main():
    global SWARM_MANAGER
    global RUN_TRACKER

    # -- Logs initialization ----------------------------------------------------- #
    file = open('logs.csv', 'w')
    writer = csv.writer(file)
    writer.writerow(['Crazyflie name', 'QTM packet timestamp (s)',
                     'QTM_x (m)', 'QTM_y (m)', 'QTM_z (m)', 'cf_yaw (°)',
                     'QTM_vx (m/s)', 'QTM_vy (m/s)', 'QTM_vz (m/s)',
                     'x_g (m)', 'y_g (m)', 'z_g (m)', 'yaw_g (°)',
                     'roll_c (°)', 'pitch_c (°)', 'yaw_rate_c (°/s)', 'thrust_c (PWM)'])

    # -- Crazyflie radio identifiers database ------------------------------------ #
    cf_radio_id = {'01': 'radio://0/81/2M/E7E7E7E701',
                   '02': 'radio://0/82/2M/E7E7E7E702',
                   '03': 'radio://0/83/2M/E7E7E7E703',
                   '04': 'radio://0/84/2M/E7E7E7E704',
                   '05': 'radio://0/85/2M/E7E7E7E705',
                   '06': 'radio://0/86/2M/E7E7E7E706',
                   '07': 'radio://0/87/2M/E7E7E7E707',
                   '08': 'radio://0/88/2M/E7E7E7E708',
                   '09': 'radio://0/89/2M/E7E7E7E709',
                   '10': 'radio://0/90/2M/E7E7E7E710'}

    # -- Flight parameters ------------------------------------------------------- #
    qtm_ip_address: str = '192.168.0.1'
    uav = Agent('Crazyflie', cf_radio_id['01'])
    uav.set_initial_position([1.0, 1.0, 0.0])
    uav.set_takeoff_height(0.4)
    uav.csv_logger = writer

    target = Robot('Cible')
    target.set_initial_position([0.0, 0.0, 0.0])
    target.csv_logger = writer

    # -- QTM connection and initial frame acquisition ---------------------------- #
    qtm_connection: QRTConnection = asyncio.get_event_loop().run_until_complete(
        qtm_tools.connect_to_qtm(qtm_ip_address))
    header, markers, timestamp = qtm_tools.frame_acquisition(qtm_connection)
    print(header.marker_count, 'markers found by QTM during initialization')
    qtm_tools.initial_uav_detection([uav], [target], markers, timestamp)

    # -- Crazyflie connection and swarm initialization procedure ----------------- #
    cflib.crtp.init_drivers()
    SWARM_MANAGER = SwarmObject()
    _ = Joystick(SWARM_MANAGER)

    uav.connect_cf()
    SWARM_MANAGER.add_agent(uav)
    SWARM_MANAGER.manual_flight_agents_list.append(uav.name)
    SWARM_MANAGER.robot_list = [target]

    # -- Real-time stream start -------------------------------------------------- #
    RUN_TRACKER = True
    asyncio.ensure_future(start_qtm_streaming(qtm_connection))
    asyncio.ensure_future(keyboard_handler())
    asyncio.get_event_loop().run_forever()

    # -- Disconnects the Crazyflies, stops the QTM stream and disconnects QTM ---- #
    uav.stop()
    uav.cf.close_link()

    asyncio.get_event_loop().run_until_complete(qtm_tools.disconnect_qtm(qtm_connection))
    file.close()


if __name__ == '__main__':
    global SWARM_MANAGER
    global RUN_TRACKER
    main()
