
import logging

# from UI.VehicleRepresentation_class import VehicleRepresentation
## The entire "Vehicles" module has to be imported for Vehicle subclasses detection.
## Otherwise, the UAV and Robot classes will not be detected in the "from_representation" method


class Vehicle:
    setup_attributes = dict(
        Name = str,
        Init_x = float,
        Init_y = float,
        Init_z = float,
        Simulated = bool
    )
    def __init__(self, **kwargs):
        self._logger = logging.getLogger(self.__class__.__name__)
        self.off_camera_count: int = 0
        for key, value in kwargs.items():
            setattr(self, key, value)
        self.check_init()

    def check_init(self):
        for setup_attribute in self.setup_attributes:
            if not hasattr(self, setup_attribute):
                error = AttributeError(f"Attribute '{setup_attribute}' is not defined")
                self._logger.error(error)
                raise error

    # def init_setup_attributes(self, **kwargs):
    #     for key in self.setup_attributes:
    #         if key not in kwargs:
    #             raise AttributeError(f"Missing '{key}' setup attribute")
    #         elif type(kwargs[key]) is not self.setup_attributes[key]:
    #             raise AttributeError(
    #                 f"Expected '{self.setup_attributes[key]}' for '{key}' setup attribute, "
    #                 f"got '{type(kwargs[key])}'"
    #             )
    #         else:
    #             setattr(self, key, kwargs[key])

    # @classmethod
    # def merge_setup_attributes(cls, child_class_instance):
    #     child_class_instance.setup_attributes = cls.setup_attributes | child_class_instance.setup_attributes

    def stop(self):
        pass

    # @classmethod
    # def from_representation(cls, representation: VehicleRepresentation) -> Self:
    #     representation_attributes = vars(representation)
    #     subclasses = cls.__subclasses__()
    #     vehicle_types = [subclass.__name__ for subclass in subclasses]
    #     try:
    #         subclass_index = vehicle_types.index(representation.vehicle_type)
    #         subclass = subclasses[subclass_index]
    #         new_instance = subclass(**representation_attributes)
    #     except ValueError:
    #         new_instance = Vehicle(**representation_attributes)
    #     return new_instance


