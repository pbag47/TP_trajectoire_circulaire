import asyncio
import qtm
import numpy as np

from agent_class import Agent
from qtm import QRTConnection
from qtm.packet import RT3DMarkerPositionNoLabel
from robot_class import Robot
from typing import List


async def connect_to_qtm(ip: str):
    connection: QRTConnection = await qtm.connect(ip)
    if connection is None:
        print(' ---- Warning ---- Error during QTM connection @', ip)
    else:
        print('QTM connected @', ip)
    return connection


def frame_acquisition(connection: QRTConnection):
    """
    Gets a single frame (the most recent) from QTM and extracts data from the received packet
    """
    frame: qtm.QRTPacket = asyncio.get_event_loop().run_until_complete(
        connection.get_current_frame(components=['3dnolabels']))
    timestamp = frame.timestamp * 10 ** -6
    headers, markers = frame.get_3d_markers_no_label()
    return headers, markers, timestamp


def initial_uav_detection(agents: List[Agent],
                          robots: List[Robot],
                          markers: List[RT3DMarkerPositionNoLabel],
                          timestamp: float):
    """
    Gathers each object (UAV or robot) with its corresponding QTM marker, based on the expected initial positions
    """

    if len(agents) + len(robots) > len(markers):
        print(' ---- Warning ---- : Too few detected makers than expected, flight disabled')
        for agt in agents:
            agt.stop()
    elif len(agents) + len(robots) < len(markers):
        print(' ---- Warning ---- : Too many detected markers than expected, flight disabled')
        for agt in agents:
            agt.stop()

    # For each UAV, searches for the nearest marker around the expected initial position,
    # updates the UAV position to match the marker,
    # and removes the marker from the list, so that 2 UAVs cannot be located on the same marker
    for agt in agents:
        d = [distance_init_pos_to_marker(agt.initial_position,
                                         [mk.x * 10 ** -3, mk.y * 10 ** -3, mk.z * 10 ** -3]) for mk in markers]
        try:
            min_d_index = d.index(min(d))
            agt.update_extpos(markers.pop(min_d_index), timestamp)
            print('UAV <', agt.name, '> found @', agt.extpos)
            if d[min_d_index] > 0.5:
                print(' ---- Warning ---- UAV <', agt.name, '> marker found too far from expected initial position')
                print('                   High risk of marker mismatch -> Flight disabled')
        except ValueError:
            break

    # For each robot, searches for the nearest marker around the expected initial position,
    # updates the robot position to match the marker,
    # and removes the marker from the list
    for rbt in robots:
        d = [distance_init_pos_to_marker(rbt.initial_position,
                                         [mk.x * 10 ** -3, mk.y * 10 ** -3, mk.z * 10 ** -3]) for mk in markers]
        try:
            min_d_index = d.index(min(d))
            rbt.update_extpos(markers.pop(min_d_index), timestamp)
            print('Robot <', rbt.name, '> found @', rbt.extpos)
        except ValueError:
            break


def uav_tracking(agents: List[Agent],
                 robots: List[Robot],
                 markers: List[RT3DMarkerPositionNoLabel],
                 timestamp: float):
    """
    Gathers each object (UAV or robot) with its corresponding marker, based on its last known state.
    """

    # The RT3DMarkerPositionNoLabel object from QTM library has an 'id' attribute, which is a unique
    # identifying integer automatically set to every marker by QTM.
    # A marker should keep the same id from a frame to another if the tracking on QTM works well
    markers_ids = [mk.id for mk in markers]

    # The first step is then to search for any previous marker describing a UAV or robot, for which the id matches
    # with one of the newly received markers. These objects position can then be directly updated with the new
    # id-matching marker
    for rbt in robots:
        try:
            index = markers_ids.index(rbt.extpos.id)
            rbt.update_extpos(markers.pop(index), timestamp)
            del markers_ids[index]
        except ValueError:
            rbt.invalid_6dof_count += 1
            if rbt.invalid_6dof_count == 1:
                print('Warning : Robot <', rbt.name, '> lost')

    for agt in agents:
        if agt.setup_finished and agt.enabled:
            try:
                index = markers_ids.index(agt.extpos.id)
                agt.update_extpos(markers.pop(index), timestamp)
                agt.send_external_position()
                del markers_ids[index]
            except ValueError:
                agt.invalid_6dof_count += 1
                if agt.invalid_6dof_count == 1:
                    print('Warning : UAV <', agt.name, '> lost')
                if agt.invalid_6dof_count == 10:
                    print(' ---- Warning ---- UAV <', agt.name, '> off camera for too long, switching the engines off')
                    agt.stop()

    # If the QTM tracking fails but the UAV or robot is still seen by the cameras, a new marker id will be created.
    # This part of the program is made to search for lost objects, by searching for new markers around their last
    # known position
    for rbt in robots:
        if rbt.invalid_6dof_count > 0:
            d = [_distance_between_markers(rbt.extpos, mk) for mk in markers]
            try:
                if min(d) < 0.1:
                    min_d_index = d.index(min(d))
                    rbt.update_extpos(markers.pop(min_d_index), timestamp)
                    rbt.invalid_6dof_count = 0
                    print('Robot <', rbt.name, '> found @', rbt.extpos)
            except ValueError:
                break

    for agt in agents:
        if agt.invalid_6dof_count > 0:
            d = [_distance_between_markers(agt.extpos, mk) for mk in markers]
            try:
                if min(d) < 0.1:
                    min_d_index = d.index(min(d))
                    agt.update_extpos(markers.pop(min_d_index), timestamp)
                    agt.send_external_position()
                    agt.invalid_6dof_count = 0
                    print('UAV <', agt.name, '> found @', agt.extpos)
            except ValueError:
                break


def distance_init_pos_to_marker(position_1: [float] * 3, position_2: [float] * 3) -> float:
    """
    Distance (m) between marker_1 position [x1(m), y1(m), z1(m)]
    and marker_2 position [x2(m), y2(m), z2(m)]
    """
    d = np.sqrt((position_1[0] - position_2[0]) ** 2
                + (position_1[1] - position_2[1]) ** 2
                + (position_1[2] - position_2[2]) ** 2)
    return d


def _distance_between_markers(marker_1: RT3DMarkerPositionNoLabel, marker_2: RT3DMarkerPositionNoLabel) -> float:
    """
    Distance (m) between marker_1 position (m) and marker_2 position (mm)

    Warning !   Internal method developed for a specific use case (units mismatch)
                User should not call this method for another purpose
    """
    d = np.sqrt((marker_1.x - marker_2.x * 10 ** -3) ** 2
                + (marker_1.y - marker_2.y * 10 ** -3) ** 2
                + (marker_1.z - marker_2.z * 10 ** -3) ** 2)
    return d


async def disconnect_qtm(connection: QRTConnection):
    if connection is not None:
        await connection.stream_frames_stop()
        connection.disconnect()
        print('QTM disconnected')
    else:
        print('QTM connection already closed')


def test_qtm_connection():
    qtm_ip_address = '192.168.0.1'
    qtm_connection = asyncio.get_event_loop().run_until_complete(connect_to_qtm(qtm_ip_address))
    header, markers, timestamp = frame_acquisition(qtm_connection)
    print('QTM packet received:')
    print('    Headers:', header)
    print('    Markers (Coordinates in mm) :', markers)
    print('    Timestamp (s) :', timestamp)
    asyncio.get_event_loop().run_until_complete(disconnect_qtm(qtm_connection))


if __name__ == '__main__':
    """
    Test de connexion à QTM.
    Dans la fonction 'test_qtm_connection()', renseignez l'adresse IP de Qualisys Track Manager.

    Comportement attendu :
    Une fois le système de Motion Capture opérationnel et le drone dans l'arène de vol, l'exécution du programme de
    test aboutit à la connection à QTM, puis un récapitulatif des marqueurs trouvés par les caméras est affiché dès
    lors qu'un paquet est reçu, et puis la déconnexion de QTM met fin à l'exécution du programme,
    """
    test_qtm_connection()
