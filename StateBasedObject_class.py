
import qtm_rt

from State_class import State

class StateBasedObject:
    def __init__(self):
        self._actual_state: State = State()
        self.measured_state: State = State()
        self.initial_state: State = State()

    def get_actual_state(self):
        return self._actual_state

    def update_measured_state(
            self,
            position_marker: qtm_rt.packet.RT3DMarkerPositionNoLabel,
            timestamp: float,
            ):
        self.measured_state.update_position(position_marker, timestamp)

