from Vehicles.Vehicle_class import Vehicle


class UAV(Vehicle):
    setup_attributes = dict(
        Antenna = str,
        Channel = str,
        Bandwidth = str,
        Address = str,
        Takeoff_z = float,
        Enabled = bool,
    )
    def __init__(self, **kwargs):
        super().merge_setup_attributes(self)
        super().__init__(**kwargs)

