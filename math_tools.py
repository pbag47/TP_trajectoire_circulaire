import numpy
from qtm_rt.packet import RT3DMarkerPositionNoLabel


def distance_xyz(marker_1: RT3DMarkerPositionNoLabel,
                 marker_2: RT3DMarkerPositionNoLabel) -> float:
    """
    Distance (m) between marker_1 position (m)
    and marker_2 position (m)
    """
    d = numpy.sqrt((marker_1.x - marker_2.x) ** 2
                   + (marker_1.y - marker_2.y) ** 2
                   + (marker_1.z - marker_2.z) ** 2)
    return d


def distance_xy(marker_1: RT3DMarkerPositionNoLabel,
                marker_2: RT3DMarkerPositionNoLabel) -> float:
    """
    Distance (m) between marker_1 position (m)
    and marker_2 position (m)
    """
    d = numpy.sqrt((marker_1.x - marker_2.x) ** 2
                   + (marker_1.y - marker_2.y) ** 2)
    return d


def gradient_xyz(
        marker_1: RT3DMarkerPositionNoLabel,
        marker_2: RT3DMarkerPositionNoLabel,
        measure_period: float, ) -> RT3DMarkerPositionNoLabel:
    gradient = RT3DMarkerPositionNoLabel(
        x=(marker_2.x * 10 ** -3 - marker_1.x) / measure_period,
        y=(marker_2.x * 10 ** -3 - marker_1.x) / measure_period,
        z=(marker_2.x * 10 ** -3 - marker_1.x) / measure_period,
        id=None,
    )
    return gradient


def velocity_xyz(velocity_coordinates: RT3DMarkerPositionNoLabel) -> float:
    v = numpy.sqrt(velocity_coordinates.x ** 2
                   + velocity_coordinates.y ** 2
                   + velocity_coordinates.z ** 2)
    return v


def velocity_xy(velocity_coordinates: RT3DMarkerPositionNoLabel) -> float:
    v = numpy.sqrt(velocity_coordinates.x ** 2 + velocity_coordinates.y ** 2)
    return v


def _distance_between_markers(marker_1: RT3DMarkerPositionNoLabel, marker_2: RT3DMarkerPositionNoLabel) -> float:
    """
    Distance (m) between marker_1 position (m) and marker_2 position (mm)

    Warning !   Internal method developed for a specific use case (units mismatch)
                User should not call this method for another purpose
    """
    d = numpy.sqrt((marker_1.x - marker_2.x * 10 ** -3) ** 2
                   + (marker_1.y - marker_2.y * 10 ** -3) ** 2
                   + (marker_1.z - marker_2.z * 10 ** -3) ** 2)
    return d