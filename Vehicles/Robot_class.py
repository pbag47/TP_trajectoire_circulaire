from Vehicles.Vehicle_class import Vehicle


class Robot(Vehicle):
    setup_attributes = Vehicle.setup_attributes + dict()
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
