import logging

from qtm.packet import RT3DMarkerPositionNoLabel


logger = logging.getLogger(__name__)


class Robot:
    def __init__(self, name: str):
        self.name:                  str = name
        self.timestamp:             float = 0.0                 # (m)
        self.initial_position:      [float] * 3 = [0, 0, 0]     # ([m, m, m])
        self.invalid_6dof_count:    int = 0
        self.position: RT3DMarkerPositionNoLabel = RT3DMarkerPositionNoLabel(x=0,  # (m)
                                                                             y=0,  # (m)
                                                                             z=0,  # (m)
                                                                             id=None,  # (positive integer, no unit)
                                                                             )
        self.csv_logger = None

    def set_initial_position(self, position: [float] * 3):
        self.initial_position = position

    def update_position(self, marker: RT3DMarkerPositionNoLabel, timestamp: float):
        if timestamp > self.timestamp:
            self.timestamp = timestamp
            self.position = RT3DMarkerPositionNoLabel(marker.x * 10 ** -3,
                                                      marker.y * 10 ** -3,
                                                      marker.z * 10 ** -3,
                                                      marker.id)
            self.log_position()
        else:
            logger.error(self.name + ' Several QTM packets received for the same timestamp')

    def log_position(self):
        self.csv_logger.writerow([self.name, self.timestamp,
                                  self.position.x, self.position.y, self.position.z, 'None',
                                  'None', 'None', 'None',
                                  'None', 'None', 'None', 'None',
                                  'None', 'None', 'None', 'None'])