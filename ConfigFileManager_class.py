import os
from typing import Any


class ConfigFileManager:
    def __init__(self, config_folder_path="config"):
        self.config_folder_path = config_folder_path

    def generate_instances(self, class_object) -> list:
        config_file_path = os.path.join(self.config_folder_path, class_object.__name__ + ".txt")
        with open(config_file_path, 'r') as file:
            header_line = file.readline()
            data_lines = file.readlines()
        header_elements = [element.strip() for element in header_line.split(",")]
        parameters_map = dict()
        for setup_attribute_name in class_object.setup_attributes.keys():
            try:
                index = header_elements.index(setup_attribute_name)
                parameters_map[setup_attribute_name] = index
            except ValueError as error:
                error.add_note(
                    f"Setup attribute '{setup_attribute_name}' not found in config file '{config_file_path}'"
                )
                raise error
        list_of_instances = []
        for line in data_lines:
            instance_arguments = dict()
            line_elements = [element.strip() for element in line.split(",")]
            for setup_attribute_name, setup_attribute_type in class_object.setup_attributes.items():
                index = parameters_map[setup_attribute_name]
                value_str = line_elements[index]
                argument_value = convert_str_to_any(value_str, setup_attribute_type)
                instance_arguments[setup_attribute_name] = argument_value
            list_of_instances.append(class_object(**instance_arguments))
        return list_of_instances

    def save_as_config_file(self, list_of_instances):
        pass


def convert_str_to_any(input_str: str, output_type: type) -> Any:
    if output_type == bool:
        output = bool(int(input_str))
    else:
        output = output_type(input_str)
    return output


def convert_any_to_str(input_any: Any) -> str:
    if type(input_any) == bool:
        output = str(int(input_any))
    else:
        output = str(input_any)
    return output