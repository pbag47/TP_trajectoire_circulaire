import pyqtgraph

from PySide6 import QtCore
from PySide6.QtGui import QPen, QColorConstants
from qtm_rt.packet import RT3DMarkerPositionNoLabel

from StateBasedObject_class import StateBasedObject


class VehicleRepresentation(StateBasedObject):
    def __init__(self,
                 vehicle_type: str,
                 name: str,
                 antenna: str,
                 channel: str,
                 bandwidth: str,
                 address: str,
                 x: float,
                 y: float,
                 z: float,
                 takeoff_z: float,
                 enabled: bool,
                 simulated: bool,
                 plot_widget: pyqtgraph.PlotWidget,
                 **kwargs,
                 ):
        super().__init__()
        self.vehicle_type: str = vehicle_type
        self.name: str = name
        self.antenna: str = antenna
        self.channel: str = channel
        self.bandwidth: str = bandwidth
        self.address: str = address
        self.takeoff_z: float = takeoff_z
        self.enabled: bool = enabled
        self.simulated: bool = simulated
        self.initial_state.position = RT3DMarkerPositionNoLabel(
            x=x,
            y=y,
            z=z,
            id=None,
        )  # State declared by the user in the UI
        self.measured_state.position = RT3DMarkerPositionNoLabel(
            x=x,
            y=y,
            z=z,
            id=None,
        )  # State measured by QTM (Real or simulated)
        self.actual_matches_measure: bool = False
        self.color = QPen(QColorConstants.Blue, 0, QtCore.Qt.PenStyle.SolidLine)
        match self.vehicle_type:
            case 'UAV':
                self.color = QPen(QColorConstants.Green, 0, QtCore.Qt.PenStyle.SolidLine)
            case 'Robot':
                self.color = QPen(QColorConstants.Red, 0, QtCore.Qt.PenStyle.SolidLine)
        self.plot_widget: pyqtgraph.PlotWidget = plot_widget
        self.scatter = self.plot_widget.plot([0], [0], symbol="x", name=self.name, pen=self.color)
        self.curve = self.plot_widget.plot([0], [0], pen=self.color)

        for key, value in kwargs.items():
            setattr(self, key, value)

    def update(self):
        if self.enabled:
            self.scatter.setData(
                [self.initial_state.position.x],
                [self.initial_state.position.y],
            )
            self.scatter.setVisible(True)
            if self.actual_matches_measure:
                self.curve.setData(
                    [self.initial_state.position.x, self.measured_state.position.x],
                    [self.initial_state.position.y, self.measured_state.position.y],
                )
                self.curve.setVisible(True)
            else:
                self.curve.setVisible(False)
        else:
            self.scatter.setVisible(False)
            self.curve.setVisible(False)