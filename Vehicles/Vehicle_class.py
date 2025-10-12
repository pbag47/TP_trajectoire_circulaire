
import logging

from StateBasedObject_class import StateBasedObject


class Vehicle(StateBasedObject):
    def __init__(self,
                 name: str,
                 ):
        super().__init__()
        self._logger = logging.getLogger(self.__class__.__name__)
        self.name: str = name
        self.off_camera_count: int = 0

    def stop(self):
        pass
