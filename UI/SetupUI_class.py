
import logging
import os.path
import pyqtgraph
import qtm_rt
import sys

from PySide6 import QtWidgets, QtGui, QtCore
from PySide6.QtWidgets import QGridLayout

import math_tools
from QTM import QTMHandler
from QTM.QTMConnection_class import QTMConnection
from QTM.QTMVirtualConnection_class import QTMVirtualConnection
from UI.VehicleRepresentation_class import VehicleRepresentation
from UI.ToggleButton_class import ToggleButton


class InstanceSelector(QtWidgets.QWidget):
    def __init__(self, multiple_selection: bool = True, parent=None):
        super(InstanceSelector, self).__init__(parent)
        self.multiple_selection: bool = multiple_selection
        self.layout = QGridLayout(self)




class SetupUI(QtWidgets.QWidget):
    def __init__(self, parameters_filename: str = 'flight_parameters.txt'):
        super().__init__()
        self._logger = logging.getLogger(__class__.__name__)
        self.qtm_handler: QTMHandler = QTMHandler()
        self.vehicle_markers: list[VehicleRepresentation] = []
        self.selected_vehicle: VehicleRepresentation | None = None
        self.parameters_filename = parameters_filename
        self.xy_plot = pyqtgraph.PlotWidget(self)

        self.read_parameters_file()

        self.main_layout = QtWidgets.QGridLayout(self)
        self.cf_selection_layout = QtWidgets.QGridLayout(self)
        self.robot_selection_layout = QtWidgets.QGridLayout(self)
        self.settings_layout = QtWidgets.QFormLayout(self)

        self.vehicle_choice_label = QtWidgets.QLabel(self)
        self.vehicle_choice_combobox = QtWidgets.QComboBox(self)
        self.separation_line_from_selection_to_init_pos = QtWidgets.QFrame(self)
        self.initial_position_label = QtWidgets.QLabel(self)
        self.initial_x_label = QtWidgets.QLabel(self)
        self.initial_x_spinbox = QtWidgets.QDoubleSpinBox(self)
        self.initial_y_label = QtWidgets.QLabel(self)
        self.initial_y_spinbox = QtWidgets.QDoubleSpinBox(self)
        self.initial_z_label = QtWidgets.QLabel(self)
        self.initial_z_spinbox = QtWidgets.QDoubleSpinBox(self)
        self.separation_line_from_init_pos_to_takeoff_z = QtWidgets.QFrame(self)
        self.takeoff_z_label = QtWidgets.QLabel(self)
        self.takeoff_z_spinbox = QtWidgets.QDoubleSpinBox(self)

        self.cf_selection_radiobuttons: list[QtWidgets.QRadioButton] = []
        self.robot_selection_checkboxes: list[QtWidgets.QCheckBox] = []

        self.simulation_toggle = ToggleButton(self)
        self.validate_button = QtWidgets.QCommandLinkButton(self)

        self.setup()
        self.qtm_handler.connection.qtm_measure.set_vehicles_list(self.vehicle_markers)
        self.qtm_handler.connection.on_packet = self.update_graph
        self.start_qtm()

    def start_qtm(self):
        print("Start QTM")
        self.qtm_handler.connection.start()

    def stop_qtm(self):
        print("Stop QTM")
        self.qtm_handler.connection.stop()

    def setup(self):
        self.resize(800, 600)
        self.setup_xy_plot()
        self.setup_settings_layout()
        self.setup_selection_layouts()
        self.setup_simulation_toggle()
        self.validate_button.setText("Valider")
        self.main_layout.addWidget(self.xy_plot, 0, 0)
        self.main_layout.addLayout(self.settings_layout, 0, 1)
        self.main_layout.addLayout(self.cf_selection_layout, 1, 0)
        self.main_layout.addWidget(self.simulation_toggle, 1, 1)
        self.main_layout.addLayout(self.robot_selection_layout, 2, 0)
        self.main_layout.addWidget(self.validate_button, 2, 1)
        self.connect_callbacks()
        self.update_vehicle_choice_combobox()

    def setup_xy_plot(self):
        self.xy_plot.showGrid(x=True, y=True)
        self.xy_plot.setTitle('Graphique X-Y')
        self.xy_plot.setLabel('bottom', 'X (m)')
        self.xy_plot.setLabel('left', 'Y (m)')
        self.xy_plot.setAspectLocked(True)
        self.xy_plot.setXRange(-2, 2)
        self.xy_plot.setYRange(-2, 2)

    def setup_settings_layout(self):
        self.separation_line_from_selection_to_init_pos.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.separation_line_from_init_pos_to_takeoff_z.setFrameShape(QtWidgets.QFrame.Shape.HLine)

        self.vehicle_choice_label.setText("Selection")
        self.initial_position_label.setText("Position initiale")
        self.initial_x_label.setText("X (m)")
        self.initial_y_label.setText("Y (m)")
        self.initial_z_label.setText("Z (m)")
        self.takeoff_z_label.setText("Hauteur de décollage (m)")

        self.initial_x_spinbox.setRange(-2, 2)
        self.initial_y_spinbox.setRange(-2, 2)
        self.initial_z_spinbox.setRange(0, 2)
        self.takeoff_z_spinbox.setRange(0, 2)

        self.settings_layout.addRow(self.vehicle_choice_label, self.vehicle_choice_combobox)
        self.settings_layout.addRow(self.separation_line_from_selection_to_init_pos)
        self.settings_layout.addRow(self.initial_position_label)
        self.settings_layout.addRow(self.initial_x_label, self.initial_x_spinbox)
        self.settings_layout.addRow(self.initial_y_label, self.initial_y_spinbox)
        self.settings_layout.addRow(self.initial_z_label, self.initial_z_spinbox)
        self.settings_layout.addRow(self.separation_line_from_init_pos_to_takeoff_z)
        self.settings_layout.addRow(self.takeoff_z_label, self.takeoff_z_spinbox)

    def setup_selection_layouts(self):
        max_number_of_widgets_per_row = 5
        cf_row_count = 0
        cf_column_count = 0
        rbt_row_count = 0
        rbt_column_count = 0
        for vehicle_marker in self.vehicle_markers:
            if vehicle_marker.vehicle_type == 'UAV':
                radio_button = QtWidgets.QRadioButton(self)
                radio_button.setText(vehicle_marker.name)
                radio_button.setChecked(vehicle_marker.enabled)
                self.cf_selection_radiobuttons.append(radio_button)
                self.cf_selection_layout.addWidget(radio_button, cf_row_count, cf_column_count)
                cf_column_count += 1
                if cf_column_count >= max_number_of_widgets_per_row:
                    cf_row_count += 1
                    cf_column_count = 0
            if vehicle_marker.vehicle_type == 'Robot':
                check_box = QtWidgets.QCheckBox(self)
                check_box.setText(vehicle_marker.name)
                check_box.setChecked(vehicle_marker.enabled)
                self.robot_selection_checkboxes.append(check_box)
                self.robot_selection_layout.addWidget(check_box, rbt_row_count, rbt_column_count)
                rbt_column_count += 1
                if rbt_column_count >= max_number_of_widgets_per_row:
                    rbt_row_count += 1
                    rbt_column_count = 0

    def setup_simulation_toggle(self):
        self.simulation_toggle.checked_state.text = "Vol réel"
        self.simulation_toggle.checked_state.button_color = QtGui.QColorConstants.Red
        self.simulation_toggle.checked_state.text_color = QtGui.QColorConstants.White

        self.simulation_toggle.unchecked_state.text = "Simulation"
        self.simulation_toggle.unchecked_state.button_color = QtGui.QColorConstants.Blue
        self.simulation_toggle.unchecked_state.text_color = QtGui.QColorConstants.White

    def connect_callbacks(self):
        self.vehicle_choice_combobox.currentIndexChanged.connect(self.vehicle_choice_callback)
        for radiobutton in self.cf_selection_radiobuttons:
            radiobutton.clicked.connect(self.vehicle_enabled_callback)
        for checkbox in self.robot_selection_checkboxes:
            checkbox.clicked.connect(self.vehicle_enabled_callback)
        self.initial_x_spinbox.valueChanged.connect(self.x_changed_callback)
        self.initial_y_spinbox.valueChanged.connect(self.y_changed_callback)
        self.initial_z_spinbox.valueChanged.connect(self.z_changed_callback)
        self.takeoff_z_spinbox.valueChanged.connect(self.z_takeoff_changed_callback)
        self.simulation_toggle.clicked.connect(self.simulation_toggle_callback)
        self.validate_button.clicked.connect(self.submit_callback)

    @QtCore.Slot()
    def vehicle_enabled_callback(self):
        for radiobutton in self.cf_selection_radiobuttons:
            vehicle_marker = [vehicle for vehicle in self.vehicle_markers if vehicle.name == radiobutton.text()][0]
            vehicle_marker.enabled = radiobutton.isChecked()
        for checkbox in self.robot_selection_checkboxes:
            vehicle_marker = [vehicle for vehicle in self.vehicle_markers if vehicle.name == checkbox.text()][0]
            vehicle_marker.enabled = checkbox.isChecked()
        self.update_vehicle_choice_combobox()

    @QtCore.Slot()
    def x_changed_callback(self):
        self.selected_vehicle.initial_state.position = qtm_rt.packet.RT3DMarkerPositionNoLabel(
            x=self.initial_x_spinbox.value(),
            y=self.selected_vehicle.initial_state.position.y,
            z=self.selected_vehicle.initial_state.position.z,
            id=self.selected_vehicle.initial_state.position.id
        )

    def y_changed_callback(self):
        self.selected_vehicle.initial_state.position = qtm_rt.packet.RT3DMarkerPositionNoLabel(
            x=self.selected_vehicle.initial_state.position.x,
            y=self.initial_y_spinbox.value(),
            z=self.selected_vehicle.initial_state.position.z,
            id=self.selected_vehicle.initial_state.position.id
        )

    def z_changed_callback(self):
        self.selected_vehicle.initial_state.position = qtm_rt.packet.RT3DMarkerPositionNoLabel(
            x=self.selected_vehicle.initial_state.position.x,
            y=self.selected_vehicle.initial_state.position.y,
            z=self.initial_z_spinbox.value(),
            id=self.selected_vehicle.initial_state.position.id
        )

    @QtCore.Slot()
    def z_takeoff_changed_callback(self):
        self.selected_vehicle.takeoff_z = self.takeoff_z_spinbox.value()

    @QtCore.Slot()
    def simulation_toggle_callback(self):
        self.stop_qtm()
        if self.simulation_toggle.isChecked():
            print("Real")
            self.qtm_handler.connection = QTMConnection(on_packet=self.update_graph)
        else:
            print("Simulation")
            self.qtm_handler.connection = QTMVirtualConnection(on_packet=self.update_graph)
            self.qtm_handler.connection.qtm_measure.set_vehicles_list(self.vehicle_markers)
        self.start_qtm()

    @QtCore.Slot(int)
    def vehicle_choice_callback(self, index: int):
        possible_choices = [vehicle for vehicle in self.vehicle_markers if vehicle.enabled]
        self.selected_vehicle = possible_choices[index]
        self.update_settings_widgets()

    @QtCore.Slot()
    def submit_callback(self):
        self.update_parameters_file()
        self.close()

    @QtCore.Slot()
    def update_vehicle_choice_combobox(self):
        names = [vehicle.name for vehicle in self.vehicle_markers if vehicle.enabled]
        self.vehicle_choice_combobox.clear()
        if names:
            self.vehicle_choice_combobox.addItems(names)

    def update_settings_widgets(self):
        if self.selected_vehicle.vehicle_type == 'UAV':
            self.takeoff_z_spinbox.setVisible(True)
            self.takeoff_z_label.setVisible(True)
            self.takeoff_z_spinbox.setValue(self.selected_vehicle.takeoff_z)
        elif self.selected_vehicle.vehicle_type == 'Robot':
            self.takeoff_z_spinbox.setVisible(False)
            self.takeoff_z_label.setVisible(False)
        initial_state = self.selected_vehicle.get_actual_state()
        self.initial_x_spinbox.setValue(initial_state.position.x)
        self.initial_y_spinbox.setValue(initial_state.position.y)
        self.initial_z_spinbox.setValue(initial_state.position.z)

    def update_graph(self, markers, _):
        self.find_nearest_markers(self.vehicle_markers, markers)
        for vehicle_representation in self.vehicle_markers:
            vehicle_representation.update()

    @staticmethod
    def find_nearest_markers(
            vehicle_representations: list[VehicleRepresentation],
            markers: list[qtm_rt.packet.RT3DMarkerPositionNoLabel],
            ):
        pending_markers = markers.copy()
        for vehicle in vehicle_representations:
            distances = [math_tools.distance_xy(vehicle.get_actual_state().position, marker) for marker in pending_markers]
            if not distances:
                vehicle.measured_state.position = qtm_rt.packet.RT3DMarkerPositionNoLabel(
                    x=0,
                    y=0,
                    z=0,
                    id=None,
                )
                vehicle.actual_matches_measure = False
                continue
            min_distance_index = distances.index(min(distances))
            marker = pending_markers[min_distance_index]
            vehicle.measured_state.position = marker
            if distances[min_distance_index] < 0.5:
                vehicle.actual_matches_measure = True
            else:
                vehicle.actual_matches_measure = False
            pending_markers.remove(marker)

    def read_parameters_file(self):
        self.vehicle_markers = []
        with open(self.parameters_filename, 'r') as file:
            _ = file.readline()
            lines = file.readlines()
            for line in lines:
                (
                    vehicle_type, name, antenna, channel, bandwidth, address,
                    x, y, z, takeoff_z, enabled, simulated
                )  = [element.strip() for element in line.split(',')]
                x = float(x)
                y = float(y)
                z = float(z)
                takeoff_z = float(takeoff_z)
                enabled = bool(int(enabled))
                simulated = bool(int(simulated))
                self.vehicle_markers.append(
                    VehicleRepresentation(
                        vehicle_type=vehicle_type,
                        name=name,
                        antenna=antenna,
                        channel=channel,
                        bandwidth=bandwidth,
                        address=address,
                        x=x,
                        y=y,
                        z=z,
                        takeoff_z=takeoff_z,
                        enabled=enabled,
                        simulated=simulated,
                        plot_widget=self.xy_plot,
                    ),
                )
        if sum([vehicle_marker.enabled for vehicle_marker in self.vehicle_markers if vehicle_marker.vehicle_type == 'UAV']) > 1:
            self._logger.warning(
                "Multiple UAVs set to enabled in the config file. "
                "However, this project only allows a single UAV to fly at a time. "
                "All UAVs are now being disabled"
            )
            for vehicle_marker in self.vehicle_markers:
                vehicle_marker.enabled = False

    def update_parameters_file(self):
        text = ['Type, Name, Init_x, Init_y, Init_z, Takeoff_z, Enabled \n']
        for vehicle_representation in self.vehicle_markers:
            initial_state = vehicle_representation.get_actual_state().position
            line = vehicle_representation.vehicle_type + ', '
            line += vehicle_representation.name + ', '
            line += vehicle_representation.antenna + ', '
            line += vehicle_representation.channel + ', '
            line += vehicle_representation.bandwidth + ', '
            line += vehicle_representation.address + ', '
            line += str(initial_state.position.x) + ', '
            line += str(initial_state.position.y) + ', '
            line += str(initial_state.position.z) + ', '
            line += str(vehicle_representation.takeoff_z) + ', '
            line += str(int(vehicle_representation.enabled)) + ', '
            line += str(int(vehicle_representation.simulated)) + '\n'
            text.append(line)
        with open(self.parameters_filename, 'w') as file:
            file.writelines(text)

    def stop(self):
        self.stop_qtm()


def test():
    settings_app = QtWidgets.QApplication([])
    parameters_file_name = os.path.join('..', 'flight_parameters.txt')
    setup_ui = SetupUI(parameters_filename=parameters_file_name)
    setup_ui.show()
    exit_code = settings_app.exec()
    setup_ui.stop()
    sys.exit(exit_code)


if __name__ == '__main__':
    test()
