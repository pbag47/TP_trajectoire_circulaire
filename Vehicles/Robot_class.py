from Vehicles.Vehicle_class import Vehicle


class Robot(Vehicle):
    setup_attributes = dict()
    def __init__(self, **kwargs):
        super().merge_setup_attributes(self)
        super().__init__(**kwargs)
