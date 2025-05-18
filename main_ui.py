import gui
import numpy
import sys
import qtm_rt

from PySide6 import QtCore
from PySide6.QtGui import QPen, QColorConstants
from PySide6.QtWidgets import QApplication
from typing import List

from agent_class import Agent
from cf_info import init_agents
from robot_class import Robot


class GraphicRepresentation:
    def __init__(self, vehicle_type: str, vehicle: Agent | Robot, radio_button, plot_widget):
        self.name = vehicle.name
        self.type = vehicle_type
        self.vehicle = vehicle
        self.plot_widget = plot_widget

        color = QPen(QColorConstants.Blue, 0, QtCore.Qt.PenStyle.SolidLine)
        if self.type == 'UAV':
            color = QPen(QColorConstants.White, 0, QtCore.Qt.PenStyle.SolidLine)
        elif self.type == 'Robot':
            color = QPen(QColorConstants.Red, 0, QtCore.Qt.PenStyle.SolidLine)

        self.marker = None
        self.marker_gr = self.plot_widget.plot([0], [0], symbol="x", name=self.name, pen=color)
        self.curve = self.plot_widget.plot([0], [0], pen=color)
        self.enabled = 0
        self.radio_button = radio_button
        self.update()

    def update(self):
        if self.enabled:
            self.marker_gr.setData([self.vehicle.initial_position[0]], [self.vehicle.initial_position[1]])
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
    def __init__(self, qtm_marker, plot_widget):
        self.marker = qtm_marker
        self.plot_widget = plot_widget
        self.gr = self.plot_widget.plot([self.marker.x * 10 ** -3], [self.marker.y * 10 ** -3],
                                        name=str(self.marker.id),
                                        symbol="o")

    def update(self, qtm_marker):
        self.marker = qtm_marker
        self.gr.setData([self.marker.x * 10 ** -3], [self.marker.y * 10 ** -3])

    def delete(self):
        self.plot_widget.removeItem(self.gr)
        del self

class Window(gui.UiMainWindow):
    def __init__(self,
                 parent=None,
                 uavs: List[Agent] = None,
                 robot: Robot = None,
                 parameters_filename=None,
                 test_mode=False):
        super().__init__(parent)
        self.filename = parameters_filename
        self.uav = None
        self.robot = None
        self.qtm_markers_gr_list = []

        self.plot.showGrid(x=True, y=True)
        self.plot.setTitle('Initial position visualizer')
        self.plot.setLabel('bottom', 'X (m)')
        self.plot.setLabel('left', 'Y (m)')
        self.plot.setXRange(-2, 2, 0.50)
        self.plot.setYRange(-2, 2, 0.50)

        self.series = []
        self.enabled_cf_radio_button_list = [self.cb_cf1, self.cb_cf2, self.cb_cf3, self.cb_cf4, self.cb_cf5,
                                             self.cb_cf6, self.cb_cf7, self.cb_cf8, self.cb_cf9, self.cb_cf10]
        for uav in uavs:
            rb = [radio_button for radio_button in self.enabled_cf_radio_button_list if
                  radio_button.objectName().endswith(uav.name)]
            self.series.append(GraphicRepresentation('UAV', uav, rb[0], plot_widget=self.plot))
        self.series.append(GraphicRepresentation('Robot', robot, self.cb_robot, plot_widget=self.plot))
        self.selected_gr = []
        self.read_parameters_file()
        self.update_combobox()
        self.connect_callbacks()
        if test_mode:
            self.init_test()
            self.test()
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

    def update_graph(self, markers):
        # _, markers = packet.get_3d_markers_no_label()

        remaining_markers = markers.copy()
        updated_markers = []
        for qtm_gr in self.qtm_markers_gr_list:
            updated_marker_found = [marker for marker in markers if marker.id == qtm_gr.marker.id]
            if updated_marker_found:
                qtm_gr.update(updated_marker_found[0])
                remaining_markers.remove(updated_marker_found[0])
                updated_markers.append(updated_marker_found[0])

        for marker in remaining_markers:
            self.qtm_markers_gr_list.append(QtmMarkersGR(marker, self.plot))
            updated_markers.append(marker)

        lost_markers = list(set([marker_gr.marker for marker_gr in self.qtm_markers_gr_list]) - set(updated_markers))
        for lost_marker in lost_markers:
            lost_marker_gr_found = [marker_gr for marker_gr in self.qtm_markers_gr_list if marker_gr.marker.id == lost_marker.id]
            if lost_marker_gr_found:
                self.qtm_markers_gr_list.remove(lost_marker_gr_found[0])
                lost_marker_gr_found[0].delete()

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

    def init_test(self):
        number_of_markers = 2
        self.offline_markers = []
        for i in range(number_of_markers):
            self.offline_markers.append(qtm_rt.packet.RT3DMarkerPositionNoLabel(x=numpy.random.randn() * 10**3,
                                                                                y=numpy.random.randn() * 10**3,
                                                                                z=0,
                                                                                id=i))

    def offline_update(self):
        for i in range(len(self.offline_markers)):
            marker = self.offline_markers[i]
            marker = qtm_rt.packet.RT3DMarkerPositionNoLabel(x=marker.x + numpy.random.randn() * 0.1,
                                                             y=marker.y + numpy.random.randn() * 0.1,
                                                             z=marker.z,
                                                             id=marker.id)
            self.offline_markers[i] = marker
        self.update_graph(self.offline_markers)

    def test(self):
        self.timer = QtCore.QTimer()
        self.timer.setInterval(20)
        self.timer.timeout.connect(self.offline_update)
        self.timer.start()


if __name__ == '__main__':
    agents_list = init_agents()
    rbt = Robot('Cible')
    app_test = QApplication(sys.argv)
    filename = 'flight_parameters.txt'
    user_window = Window(uavs=agents_list, robot=rbt, parameters_filename=filename, test_mode=True)
    exit_code = app_test.exec()
    sys.exit(exit_code)
