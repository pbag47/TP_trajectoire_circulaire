
import logging
import qtm_rt
import math_tools


class State:
    def __init__(self):
        self._logger = logging.getLogger(self.__class__.__name__)
        self.last_update_timestamp: float = 0.0
        self.update_period: float = 1.0
        self.position: qtm_rt.packet.RT3DMarkerPositionNoLabel = qtm_rt.packet.RT3DMarkerPositionNoLabel(
            x=0,
            y=0,
            z=0,
            id=None,
        )
        self.velocity: qtm_rt.packet.RT3DMarkerPositionNoLabel = qtm_rt.packet.RT3DMarkerPositionNoLabel(
            x=0,
            y=0,
            z=0,
            id=None,
        )
        self.acceleration: qtm_rt.packet.RT3DMarkerPositionNoLabel = qtm_rt.packet.RT3DMarkerPositionNoLabel(
            x=0,
            y=0,
            z=0,
            id=None,
        )

    def update_position(self,
                        new_position: qtm_rt.packet.RT3DMarkerPositionNoLabel,
                        timestamp: float,
                        ):
        if timestamp > self.last_update_timestamp:
            self.update_period = timestamp - self.last_update_timestamp
            self._update_velocity(new_position)
            self.last_update_timestamp = timestamp
            self.position = new_position
        else:
            self._logger.error('Several QTM packets received for the same timestamp')

    def _update_velocity(self, new_position_marker: qtm_rt.packet.RT3DMarkerPositionNoLabel):
        new_velocity = math_tools.gradient_xyz(self.position, new_position_marker, self.update_period)
        self._update_acceleration(new_velocity)
        self.velocity = new_velocity

    def _update_acceleration(self, new_velocity_marker: qtm_rt.packet.RT3DMarkerPositionNoLabel):
        self.acceleration = math_tools.gradient_xyz(self.velocity, new_velocity_marker, self.update_period)
