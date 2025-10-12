
import logging
import qtm_rt
import random
import threading
import time

from typing import Callable

from UI.VehicleRepresentation_class import VehicleRepresentation
from Vehicles.Vehicle_class import Vehicle


class QTMVirtualMeasure:
    def __init__(self,
                 vehicles: list[Vehicle | VehicleRepresentation] | None = None,
                 standard_deviation: float = 0.005,
                 refresh_rate: int = 25,  # Hz  (25)
                 ):
        self._logger = logging.getLogger(self.__class__.__name__)
        if vehicles is None:
            vehicles = []
        self.vehicles = vehicles
        self.standard_deviation = standard_deviation
        self.refresh_rate = refresh_rate
        self.stop_flag: threading.Event = threading.Event()
        self.thread: threading.Thread | None = None

    def set_vehicles_list(self, vehicles: list[Vehicle | VehicleRepresentation]):
        self.vehicles = vehicles

    async def stream_frames(self, components: list[str], on_packet: Callable):
        self.thread = threading.Thread(
            target=self._main,
            args=(on_packet,),
            daemon=True,
            name="QTM virtual measure thread"
        )
        self.thread.start()

    def _main(self, on_packet: Callable):
        self._logger.info("QTM stream started")
        while not self.stop_flag.is_set():
            time.sleep(1/self.refresh_rate)
            timestamp = time.time()
            headers = dict()
            markers = self.apply_measurement_noise(self.vehicles)
            on_packet(headers, markers, timestamp)
        self._logger.info("QTM stream stopped")

    def apply_measurement_noise(self,
                                vehicles_list: list[Vehicle | VehicleRepresentation],
                                ) -> list[qtm_rt.packet.RT3DMarkerPositionNoLabel]:
        new_markers_list = []
        for vehicle in vehicles_list:
            actual_position = vehicle.get_actual_state().position
            new_measured_position = qtm_rt.packet.RT3DMarkerPositionNoLabel(
                x=actual_position.x + random.normalvariate(0, self.standard_deviation),
                y=actual_position.y + random.normalvariate(0, self.standard_deviation),
                z=actual_position.z + random.normalvariate(0, self.standard_deviation),
                id=vehicle.measured_state.position.id,
            )
            new_markers_list.append(new_measured_position)
        return new_markers_list

    async def stream_frames_stop(self):
        self._logger.info("QTM stream stop request")
        self.stop_flag.set()
        self.thread.join(timeout=5)
