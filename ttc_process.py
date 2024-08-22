import time
import json
import requests
from datetime import datetime, timedelta
import geojson
from shapely.geometry import shape, Point
import get_data
import preprocessing
from safety_evaluation import SSM
from convert_xy_to_gis import gps_lin_ll2xy, gps_lin


def ttcProcess():
    """
    The complete process of calculating ttc and store the result.
    """
    # get geo json data
    with open("ca_martinez.geojson") as f:
        geo_data = geojson.load(f)
    filtered_features = [
        feature
        for feature in geo_data["features"]
        if feature["properties"]["type_names"] == "Crossing"
    ]
    # create a new geojson only including filtered features
    filtered_geojson = {"type": "FeatureCollection", "features": filtered_features}

    # set start time
    start_time_str = "2024-05-09 17:33:00"
    start_time = datetime.strptime(start_time_str, "%Y-%m-%d %H:%M:%S")
    # end_time is 5s later than start_time
    interval = timedelta(seconds=5)
    end_time = start_time + interval
    one_day_end = bool(False)
    one_day_seconds = 60 * 60 * 24

    results = []
    end_day_time = datetime.strptime("2024-05-09 17:34:00", "%Y-%m-%d %H:%M:%S")
    while start_time < end_day_time:  # one day loop
        while 1:  # continue get data until response is not empty
            neighbour_data = get_data.FetchNeighbourData(
                start_time=start_time, end_time=end_time
            )
            ego_data = get_data.FetchEgoVehicleData(
                start_time=start_time, end_time=end_time
            )
            if len(neighbour_data["Response"]) != 0 and len(ego_data["Response"]) != 0:
                break
            else:
                start_time += interval
                end_time += interval
                if start_time >= end_day_time:  # one day loop
                    one_day_end = True
                    break
        if one_day_end == True:
            break

        print(datetime.strftime(start_time, "%Y-%m-%d %H:%M:%S"))

        # start calculation
        car = SSM(5.272, 2.345)  # length and width of the car

        for pedestrian_dict in neighbour_data["Response"]:
            result = {}
            ego_dict = get_data.GetCorrespondingEgoData(
                ego_vehicle_dataset=ego_data, pedestrian_data=pedestrian_dict
            )
            ego_inner = json.loads(ego_dict["Json"])
            pedestrian_inner = json.loads(pedestrian_dict["Json"])

            pdst_gis_coord = {
                "latitude": pedestrian_inner["Latitude"],
                "longitude": pedestrian_inner["Longitude"],
            }
            pdst_point = gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=pdst_gis_coord)
            pdst_point = Point(pdst_point.x, pdst_point.y)

            # add necessary information to the result
            result["PedestrianId"] = pedestrian_inner["VehicleId"]
            result["Chid"] = pedestrian_dict["Chid"]
            result["Time"] = pedestrian_dict["Time"]

            # whether pedestrian is near a crossing
            is_near_crossing, crossing_id = preprocessing.NearCrossing(
                filtered_geojson, pdst_point
            )
            if is_near_crossing:
                result["NearCrossing"] = 1
                result["CrossingId"] = crossing_id
            else:
                result["NearCrossing"] = 0
                result["CrossingId"] = None

            # judge whether their ideal path will intersect and find out the conflict point
            is_satisfied, conflict_point = preprocessing.ConflictPoint(
                ego_vehicle=ego_inner, pedestrian=pedestrian_inner
            )

            if is_satisfied == True:
                ego_front_gis_coord = {
                    "latitude": ego_inner["FrontLat"],
                    "longitude": ego_inner["FrontLong"],
                }
                ego_front_point = gps_lin_ll2xy(
                    gps_lin=gps_lin, lat_lon=ego_front_gis_coord
                )
                ego_front_point = Point(ego_front_point.x, ego_front_point.y)

                result["TTC"] = car.calculate_ttc(
                    conflict_point=conflict_point,
                    pedestrian_point=pdst_point,
                    ego_front_point=ego_front_point,
                    pedestrian_speed=pedestrian_inner["Speed"],
                    ego_speed=ego_inner["Speed"],
                )
            else:
                result["TTC"] = None
            results.append(result)

        start_time += interval
        end_time += interval

    # one day loop

    with open("result.json", "w") as json_file:
        json.dump(results, json_file)
