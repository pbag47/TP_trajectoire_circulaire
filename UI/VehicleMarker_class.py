from PySide6 import QtCore
from PySide6.QtGui import QPen, QColorConstants


class VehicleMarker:
    def __init__(self,
                 vehicle_type: str,
                 name: str,
                 init_x: float,
                 init_y: float,
                 init_z: float,
                 takeoff_z: float,
                 enabled: bool,
                 ):
        self.vehicle_type: str = vehicle_type
        self.name: str = name
        self.init_x: float = init_x
        self.init_y: float = init_y
        self.init_z: float = init_z
        self.takeoff_z: float = takeoff_z
        self.enabled: bool = enabled

        self.qtm_marker = None

        self.color = QPen(QColorConstants.Blue, 0, QtCore.Qt.PenStyle.SolidLine)
        if self.vehicle_type == 'UAV':
            self.color = QPen(QColorConstants.Green, 0, QtCore.Qt.PenStyle.SolidLine)
        elif self.vehicle_type == 'Robot':
            self.color = QPen(QColorConstants.Red, 0, QtCore.Qt.PenStyle.SolidLine)
