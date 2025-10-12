import logging
import qtm_rt

import math_tools

from QTM.QTMConnection_class import QTMConnection
from QTM.QTMVirtualConnection_class import QTMVirtualConnection
from QTM.QTMVirtualMeasure_class import QTMVirtualMeasure
from Vehicles import Vehicle


class QTMHandler:
    def __init__(self,
                 off_camera_max_count: int = 10,
                 ):
        self._logger = logging.getLogger(self.__class__.__name__)
        self.connection: QTMConnection | QTMVirtualConnection = QTMVirtualConnection(
            qtm_virtual_measure=QTMVirtualMeasure())
        self.off_camera_max_count: int = off_camera_max_count

    def initial_vehicle_detection(self,
                                  vehicles: list[Vehicle],
                                  markers: list[qtm_rt.packet.RT3DMarkerPositionNoLabel],
                                  timestamp: float,
                                  ):
        if len(markers) != len(vehicles):
            error = ValueError(
                'Expected ' + str(len(vehicles)) + ' markers, '
                'but ' + str(len(markers)) + ' markers were received from QTM'
            )
            self._logger.error(error, stack_info=True, exc_info=True)
            for vehicle in vehicles:
                vehicle.stop()
            raise error

        for marker in markers:
            pending_vehicles = [vehicle for vehicle in vehicles if vehicle.measured_state.position.id is not None]
            distances = [math_tools.distance_xy(vehicle.initial_state.position, marker) for vehicle in pending_vehicles]
            try:
                min_distance_index = distances.index(min(distances))
                vehicle = pending_vehicles[min_distance_index]
                vehicle.update_measured_state(marker, timestamp)
                self._logger.info(
                    vehicle.name + ' found @ ' + str([
                        round(vehicle.measured_state.position.x, 2),
                        round(vehicle.measured_state.position.y, 2),
                        round(vehicle.measured_state.position.z, 2),
                    ])
                )
                if distances[min_distance_index] > 0.5:
                    self._logger.error(vehicle.name + ' marker found too far from its expected initial position')
                    vehicle.stop()
            except ValueError:
                break

    def tracking(self,
                 vehicles: list[Vehicle],
                 markers: list[qtm_rt.packet.RT3DMarkerPositionNoLabel],
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
        for vehicle in vehicles:
            try:
                index = markers_ids.index(vehicle.measured_state.position.id)
                vehicle.update_measured_state(markers.pop(index), timestamp)
                del markers_ids[index]
                vehicle.off_camera_count = 0
            except ValueError:
                vehicle.off_camera_count += 1
                if vehicle.off_camera_count == 1:
                    self._logger.warning(vehicle.name + ' lost')
                if vehicle.off_camera_count == self.off_camera_max_count:
                    self._logger.error(vehicle.name + ' off camera for too long, emergency stop triggered')
                    vehicle.stop()

        # If the QTM tracking fails but the vehicle is still seen by the cameras, a new marker id will be created.
        # This part of the program is made to search for lost objects, by searching for new markers around their last
        # known position
        for vehicle in vehicles:
            if vehicle.off_camera_count > 0:
                distances = [math_tools.distance_xyz(vehicle.measured_state.position, marker) for marker in markers]
                try:
                    if min(distances) < 0.1:
                        min_distance_index = distances.index(min(distances))
                        vehicle.update_measured_state(markers.pop(min_distance_index), timestamp)
                        vehicle.off_camera_count = 0
                        self._logger.info(vehicle.name + ' found @ ' + str(vehicle.measured_state.position))
                except ValueError:
                    break
