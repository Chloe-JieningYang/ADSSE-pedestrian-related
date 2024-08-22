import math
from shapely.geometry import Point, LineString


class SSM:
    def __init__(self, vehicle_length, vehicle_width):
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width

    def calculate_ttc(
        self,
        conflict_point: Point,
        pedestrian_point: Point,
        ego_front_point: Point,
        pedestrian_speed: float,
        ego_speed: float,
    ):
        """
        Calculate the Time to Collision (TTC) based on distances and velocities.

        parameters:

        return:
        float: The calculated TTC value.
        """
        ttc = float(0.0)
        if ego_speed < 1e-7 or pedestrian_speed < 1e-7:
            ttc = None
            return ttc
        t_vehicle = conflict_point.distance(ego_front_point) / ego_speed
        t_pedestrian = conflict_point.distance(pedestrian_point) / pedestrian_speed
        if t_vehicle < t_pedestrian:
            # vehicle passes first
            ttc = t_pedestrian
        else:
            ttc = max(
                (conflict_point.distance(pedestrian_point) + self.vehicle_width / 2)
                / pedestrian_speed,
                conflict_point.distance(ego_front_point) / ego_speed,
            )

        return ttc

    def calculate_pet(tv1, tv2, tp1, tp2):
        """
        Calculate the Post-encroachment time (PET) based on the times when the vehicle and pedestrian enter and leave the conflict zone.
        Notice: The parameters should be measured precisely, or they can be estimated by the velocity, acceleration and direction of ego-vehicle and pedestrian.

        Parameters:
        tv1 (float): Time when the vehicle enters the conflict zone.
        tv2 (float): Time when the vehicle leaves the conflict zone.
        tp1 (float): Time when the pedestrian enters the conflict zone.
        tp2 (float): Time when the pedestrian leaves the conflict zone.

        Returns:
        float: The calculated PET value.
        """
        if tv2 < tp1:  # Vehicle passes first
            pet = tp1 - tv2
        else:  # Pedestrian passes first
            pet = tv1 - tp2

        return pet

    def calculate_DRS(v, tr, D) -> float:
        """
        Calculate the decleration rate required to stop (DRS) when the ego-vehicle needs to give way to pedestrian.

        Parameters:
        v  (float): Ego-vehicle velocity.
        tr (float): The response time of the driver or the ADS.
        D  (float): The distance between the pedestrian and the vehicle.
        """

        DRS = v**2 / (2 * (D - v * tr))
        return DRS
