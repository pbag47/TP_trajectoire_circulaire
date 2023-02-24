import asyncio
import gui
import numpy
import qasync
import sys

from agent_class import Agent
from cf_info import init_agents
from PyQt5 import QtCore
from PyQt5.QtGui import QPen
from PyQt5.QtWidgets import QApplication, QMainWindow
from qwt import QwtPlot, QwtPlotCurve, QwtPlotMarker, QwtPlotGrid, QwtSymbol
from robot_class import Robot
from typing import List, Union


class GraphicRepresentation:
    def __init__(self, vehicle_type: str, vehicle: Union[Agent, Robot], radio_button):
        self.name = vehicle.name
        self.type = vehicle_type
        self.vehicle = vehicle

        color = QPen(QtCore.Qt.blue)
        if self.type == 'UAV':
            color = QPen(QtCore.Qt.black)
        elif self.type == 'Robot':
            color = QPen(QtCore.Qt.red)

        self.marker = None
        self.marker_gr = QwtPlotMarker(self.name)
        self.marker_gr.setValue(0, 0)
        symbol = QwtSymbol(QwtSymbol.XCross)
        symbol.setSize(10, 10)
        self.marker_gr.setSymbol(symbol)
        self.curve = QwtPlotCurve(self.name + ' radius')
        self.curve.setPen(color)
        self.curve.setStyle(QwtPlotCurve.Lines)
        self.enabled = 0
        self.radio_button = radio_button
        self.update()

    def update(self):
        if self.enabled:
            self.marker_gr.setValue(self.vehicle.initial_position[0], self.vehicle.initial_position[1])
            self.marker_gr.setVisible(True)
            if self.marker:
                self.curve.setData([self.vehicle.initial_position[0], self.marker.x * 10 ** -3],
                                   [self.vehicle.initial_position[1], self.marker.y * 10 ** -3])
                self.curve.setVisible(True)
            else:
                self.curve.setVisible(False)
        else:
            self.marker_gr.setVisible(False)
            self.curve.setVisible(False)


class QtmMarkersGR:
    def __init__(self, qtm_marker):
        self.marker = qtm_marker
        self.gr = QwtPlotMarker(str(qtm_marker.id))
        symbol = QwtSymbol(QwtSymbol.Ellipse)
        symbol.setSize(10, 10)
        self.gr.setSymbol(symbol)
        self.gr.setValue(self.marker.x * 10 ** -3, self.marker.y * 10 ** -3)

    def erase(self):
        self.gr.detach()
        del self


class Window(QMainWindow, gui.UiMainWindow):
    def __init__(self, parent=None, uavs: List[Agent] = None, robot: Robot = None, parameters_filename=None):
        super().__init__(parent)
        self.filename = parameters_filename
        self.setup_ui(self)
        self.uav = None
        self.robot = None
        self.qtm_markers_gr_list = []

        grid = QwtPlotGrid()
        grid.setPen(QPen(QtCore.Qt.black, 0, QtCore.Qt.DotLine))
        grid.attach(self.plot)

        self.plot.setTitle('Initial position visualizer')
        self.plot.setAxisTitle(QwtPlot.xBottom, 'X (m)')
        self.plot.setAxisTitle(QwtPlot.yLeft, 'Y (m)')
        self.plot.setAxisScale(QwtPlot.xBottom, -2, 2, 0.50)
        self.plot.setAxisScale(QwtPlot.yLeft, -2, 2, 0.50)

        self.series = []
        self.enabled_cf_radio_button_list = [self.cb_cf1, self.cb_cf2, self.cb_cf3, self.cb_cf4, self.cb_cf5,
                                             self.cb_cf6, self.cb_cf7, self.cb_cf8, self.cb_cf9, self.cb_cf10]
        for uav in uavs:
            rb = [radio_button for radio_button in self.enabled_cf_radio_button_list if
                  radio_button.objectName().endswith(uav.name)]
            self.series.append(GraphicRepresentation('UAV', uav, rb[0]))
        self.series.append(GraphicRepresentation('Robot', robot, self.cb_robot))
        for gr in self.series:
            gr.marker_gr.attach(self.plot)
            gr.curve.attach(self.plot)

        self.selected_gr = []
        self.read_parameters_file()
        self.update_combobox()
        self.connect_callbacks()
        self.show()

    def connect_callbacks(self):
        self.cf_choice.currentIndexChanged.connect(self.cf_choice_callback)
        for rb in self.enabled_cf_radio_button_list:
            rb.toggled.connect(self.cf_enabled_callback)
        self.cb_robot.clicked.connect(self.cf_enabled_callback)
        self.x.valueChanged.connect(self.x_changed_callback)
        self.y.valueChanged.connect(self.y_changed_callback)
        self.z.valueChanged.connect(self.z_changed_callback)
        self.z_takeoff.valueChanged.connect(self.z_takeoff_changed_callback)
        self.valider.clicked.connect(self.submit_callback)

    def read_parameters_file(self):
        with open(self.filename, 'r') as file:
            lines = file.readlines()
            for line in lines:
                str_values = [element.strip() for element in line.split(',')]
                for gr in self.series:
                    if gr.vehicle.name == str_values[1]:
                        gr.vehicle.set_initial_position([float(str_values[2]),
                                                         float(str_values[3]),
                                                         float(str_values[4])])
                        if gr.type == 'UAV':
                            gr.vehicle.set_takeoff_height(float(str_values[5]))
                        gr.enabled = int(str_values[6])

        uav_gr = [gr for gr in self.series if gr.type == 'UAV']
        if sum([gr.enabled for gr in uav_gr]) > 1:
            for gr in uav_gr:
                gr.enabled = False

        for gr in self.series:
            gr.radio_button.setChecked(gr.enabled)

    def cf_choice_callback(self, index):
        self.selected_gr = [gr for gr in self.series if gr.name == self.cf_choice.itemText(index)]
        if self.selected_gr:
            self.update_parameters_ui()

    def update_parameters_ui(self):
        if self.selected_gr[0].type == 'UAV':
            self.x.setRange(self.selected_gr[0].vehicle.x_boundaries[0], self.selected_gr[0].vehicle.x_boundaries[1])
            self.y.setRange(self.selected_gr[0].vehicle.y_boundaries[0], self.selected_gr[0].vehicle.y_boundaries[1])
            self.z.setRange(self.selected_gr[0].vehicle.z_boundaries[0], self.selected_gr[0].vehicle.z_boundaries[1])
            self.z_takeoff.setRange(self.selected_gr[0].vehicle.z_boundaries[0],
                                    self.selected_gr[0].vehicle.z_boundaries[1])
            self.z_takeoff.setVisible(True)
            self.label_z_takeoff.setVisible(True)
            self.z_takeoff.setValue(self.selected_gr[0].vehicle.takeoff_height)
        elif self.selected_gr[0].type == 'Robot':
            self.x.setRange(-2, 2)
            self.y.setRange(-2, 2)
            self.z.setRange(-2, 2)
            self.z_takeoff.setVisible(False)
            self.label_z_takeoff.setVisible(False)

        self.x.setValue(self.selected_gr[0].vehicle.initial_position[0])
        self.y.setValue(self.selected_gr[0].vehicle.initial_position[1])
        self.z.setValue(self.selected_gr[0].vehicle.initial_position[2])

    def x_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].vehicle.initial_position[0] = self.x.value()

    def y_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].vehicle.initial_position[1] = self.y.value()

    def z_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].vehicle.initial_position[2] = self.z.value()

    def z_takeoff_changed_callback(self):
        if self.selected_gr:
            self.selected_gr[0].vehicle.set_takeoff_height(self.z_takeoff.value())

    def cf_enabled_callback(self):
        for gr in self.series:
            gr.enabled = int(gr.radio_button.isChecked())
        self.update_combobox()

    def update_combobox(self):
        names = [gr.name for gr in self.series if gr.enabled]
        self.cf_choice.clear()
        if names:
            self.cf_choice.addItems(names)
            if not self.selected_gr:
                valid_gr = [gr for gr in self.series if gr.enabled]
                self.selected_gr = [valid_gr[0]]
            self.update_parameters_ui()

    def submit_callback(self):
        self.update_parameters_file()
        for gr in self.series:
            if gr.enabled and gr.type == 'UAV':
                self.uav = gr.vehicle
            if gr.enabled and gr.type == 'Robot':
                self.robot = gr.vehicle
        self.close()

    def update_parameters_file(self):
        text = ['Type, Name, Init_x, Init_y, Init_z, Takeoff_z, Enabled \n']
        for gr in self.series:
            line = gr.type + ', '
            line += gr.name + ', '
            line += str(gr.vehicle.initial_position[0]) + ', '
            line += str(gr.vehicle.initial_position[1]) + ', '
            line += str(gr.vehicle.initial_position[2]) + ', '
            if gr.type == 'UAV':
                line += str(gr.vehicle.takeoff_height) + ', '
            elif gr.type == 'Robot':
                line += str(0.0) + ', '
            line += str(gr.enabled) + ' \n'
            text.append(line)
        with open(self.filename, 'w') as file:
            file.writelines(text)

    def update_graph(self, packet):
        _, markers = packet.get_3d_markers_no_label()

        for qtm_gr in self.qtm_markers_gr_list:
            qtm_gr.erase()

        self.qtm_markers_gr_list = []
        for marker in markers:
            qtm_gr = QtmMarkersGR(marker)
            qtm_gr.gr.attach(self.plot)
            self.qtm_markers_gr_list.append(qtm_gr)

        # For each enabled UAV, finds the nearest QTM marker
        for gr in self.series:
            if gr.enabled and markers:
                d = [numpy.sqrt((gr.vehicle.initial_position[0] - (marker.x * 10 ** -3)) ** 2
                                + (gr.vehicle.initial_position[1] - (marker.y * 10 ** -3)) ** 2)
                     for marker in markers]
                index = d.index(min(d))
                if (d[index] <= 0.5 and gr.type == 'UAV') or gr.type == 'Robot':
                    gr.marker = markers[index]
                else:
                    gr.marker = None
            else:
                gr.marker = None
            gr.update()

        self.plot.replot()


if __name__ == '__main__':
    agents_list = init_agents()
    rbt = Robot('Cible')
    app_test = QApplication(sys.argv)
    filename = 'flight_parameters.txt'
    User_window = Window(uavs=agents_list, robot=rbt, parameters_filename=filename)
    q_loop = qasync.QEventLoop(app_test)
    asyncio.set_event_loop(q_loop)
    q_loop.run_forever()
