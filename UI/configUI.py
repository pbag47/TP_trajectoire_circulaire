
import sys

from functools import partial
from PySide6 import QtWidgets, QtCore

from ConfigFileManager_class import ConfigFileManager
from Vehicles import Vehicle, UAV, Robot


class ConfigWindow(QtWidgets.QWidget):
    def __init__(self, class_types: dict, instances: dict, parent=None):
        super(ConfigWindow, self).__init__(parent)
        self.class_types: dict = class_types
        self.instances: dict = instances
        self.layout = QtWidgets.QVBoxLayout(self)
        self.class_selector: QtWidgets.QComboBox = QtWidgets.QComboBox()
        self.main_panel_layout = QtWidgets.QGridLayout()
        self.layout.addWidget(self.class_selector)
        self.layout.addLayout(self.main_panel_layout)
        self.type_widget_match = dict(
            int = QtWidgets.QSpinBox,
            float = QtWidgets.QDoubleSpinBox,
            bool = QtWidgets.QCheckBox,
            str = QtWidgets.QLineEdit,
        )
        self.update_selector()
        self.class_selector.currentTextChanged.connect(self.update_main_panel)
        self.update_main_panel(self.class_selector.currentText())

    def update_selector(self):
        self.class_selector.clear()
        self.class_selector.addItems([key for key in self.class_types.keys()])

    def clear_main_panel(self):
        while self.main_panel_layout.count():
            element = self.main_panel_layout.takeAt(0)
            if element.widget():
                element.widget().deleteLater()

    @QtCore.Slot(str)
    def update_main_panel(self, class_name: str):
        self.clear_main_panel()
        class_type = self.class_types[class_name]
        row = 0
        headers: list[QtWidgets.QWidget] = []
        widget_types = dict()
        col = 0
        for attribute_name, attribute_type in class_type.setup_attributes.items():
            header_widget = QtWidgets.QLabel(attribute_name)
            headers.append(header_widget)
            self.main_panel_layout.addWidget(header_widget, row, col)
            widget_types[attribute_name] = self.type_widget_match[attribute_type.__name__]
            col += 1

        for instance in self.instances[class_name]:
            row += 1
            col = 0
            for attribute_name, widget_type in widget_types.items():
                value = getattr(instance, attribute_name)
                widget = widget_type()
                try:
                    widget.setValue(value)
                    widget.valueChanged.connect(partial(instance.edit_setup_attribute, attribute_name))
                except AttributeError:
                    try:
                        widget.setText(value)
                        widget.textChanged.connect(partial(instance.edit_setup_attribute, attribute_name))
                    except TypeError:
                        widget.setChecked(value)
                        widget.stateChanged.connect(partial(format_int_as_bool, instance, attribute_name))
                self.main_panel_layout.addWidget(widget, row, col)
                col += 1


@QtCore.Slot(Vehicle, str, int)
def format_int_as_bool(receiver_instance, attribute_name, int_value):
    if int_value:
        receiver_instance.edit_setup_attribute(attribute_name, True)
    else:
        receiver_instance.edit_setup_attribute(attribute_name, False)


def main():
    config_manager = ConfigFileManager()
    list_of_uav = config_manager.generate_instances(class_object=UAV)
    list_of_robots = config_manager.generate_instances(class_object=Robot)
    classes_dict = {UAV.__name__: UAV, Robot.__name__: Robot}
    instances_dict = {UAV.__name__: list_of_uav, Robot.__name__: list_of_robots}
    settings_app = QtWidgets.QApplication([])
    c = ConfigWindow(classes_dict, instances_dict)
    c.show()
    exit_code = settings_app.exec()
    config_manager.save_config([list_of_uav, list_of_robots])
    sys.exit(exit_code)


if __name__ == "__main__":
    main()