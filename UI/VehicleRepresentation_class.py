
import pyqtgraph

from PySide6 import QtCore
from PySide6.QtGui import QPen, QColorConstants
from qtm_rt.packet import RT3DMarkerPositionNoLabel

from StateBasedObject_class import StateBasedObject


class VehicleRepresentation(StateBasedObject):
    def __init__(self,
                 vehicle_type: str,
                 name: str,
                 x: float,
                 y: float,
                 z: float,
                 takeoff_z: float,
                 enabled: bool,
                 plot_widget: pyqtgraph.PlotWidget,
                 ):
        super().__init__()
        self.vehicle_type: str = vehicle_type
        self.name: str = name
        self.takeoff_z: float = takeoff_z
        self.enabled: bool = enabled
        self._actual_state.position = RT3DMarkerPositionNoLabel(
            x=x,
            y=y,
            z=z,
            id=None,
        )
        self.measured_state.position = RT3DMarkerPositionNoLabel(
            x=x,
            y=y,
            z=z,
            id=None,
        )
        self.actual_matches_measure: bool = False
        self.color = QPen(QColorConstants.Blue, 0, QtCore.Qt.PenStyle.SolidLine)
        if self.vehicle_type == 'UAV':
            self.color = QPen(QColorConstants.Green, 0, QtCore.Qt.PenStyle.SolidLine)
        elif self.vehicle_type == 'Robot':
            self.color = QPen(QColorConstants.Red, 0, QtCore.Qt.PenStyle.SolidLine)
        self.plot_widget: pyqtgraph.PlotWidget = plot_widget
        self.scatter = self.plot_widget.plot([0], [0], symbol="x", name=self.name, pen=self.color)
        self.curve = self.plot_widget.plot([0], [0], pen=self.color)

    def update(self):
        if self.enabled:
            self.scatter.setData([self._actual_state.position.x], [self._actual_state.position.y])
            self.scatter.setVisible(True)
            if self.actual_matches_measure:
                self.curve.setData([self._actual_state.position.x, self.measured_state.position.x],
                                   [self._actual_state.position.y, self.measured_state.position.y])
                self.curve.setVisible(True)
            else:
                self.curve.setVisible(False)
        else:
            self.scatter.setVisible(False)
            self.curve.setVisible(False)
