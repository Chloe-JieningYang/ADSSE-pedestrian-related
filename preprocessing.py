import numpy as np
import math
import geopandas
import json
from shapely.geometry import Point, LineString
from convert_xy_to_gis import gps_lin_ll2xy, gps_lin
import matplotlib.pyplot as plt


def CalculateDistanceByHaversine(lat1: float, lon1: float, lat2: float, lon2: float):
    """
    Calculate the distance between two spots marked with gis coordinate

    parameters: spot1 latitude, spot1 longtitude, spot2 latitude, spot2 longtitude
    """
    R = 6371393  # unit: meter
    Pi = math.pi

    a = (math.sin(math.radians((lat2 - lat1) / 2))) ** 2
    b = (
        math.cos(math.radians(lat1))
        * math.cos(math.radians(lat2))
        * (math.sin(math.radians((lon2 - lon1) / 2))) ** 2
    )
    d = 2 * R * math.asin((a + b) ** 0.5)

    return d


def CalculateDistanceToCrossing(crossing_geo: json, point_coords: Point):
    """
    Calculate the shortest distance from one coordinate to all the crossings

    parameters: geojson file only containing crossings, point coordinate (x,y): Point

    return: min_distance: float, closest_crossing: feature
    """
    min_distance = float("inf")
    closest_crossing = None
    for feature in crossing_geo["features"]:
        line_coords = feature["geometry"]["coordinates"]
        line_xy = []
        for coord in line_coords:
            lon_lat = {"longitude": coord[0], "latitude": coord[1]}
            line_xy.append(gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=lon_lat))
        line = LineString(Point(p.x, p.y) for p in line_xy)

        distance = point_coords.distance(line)
        if distance < min_distance:
            min_distance = distance
            closest_crossing = feature
    return min_distance, closest_crossing


def NearCrossing(crossing_geo: json, point_xy: Point):
    """
    Whether the pedestrian is near the crossing.

    Return:bool, crossing id or None
    """
    min_distance, closest_crossing = CalculateDistanceToCrossing(crossing_geo, point_xy)
    if min_distance < 50 and closest_crossing != None:
        return True, closest_crossing["properties"]["id"]
    else:
        return False, None


def ConflictPoint(ego_vehicle: dict, pedestrian: dict) -> bool:
    """
    Judge whether the positions of ego vehicle and pedetrian satisfy TTC calculating conditions

    Parameters:
    ego_vehicle: ego_response['Response'][i]['Json'], dict of ego_vehicle information, eg:
    {"Longitude":-122.12858376770639,"Latitude":37.994154054077832,"Speed":6.679798923845544,"Direction":160.95483398600914,"Acceleration":9.9429663805647372,"Steer":0.01955,"Brake":0.0,"FrontLat":37.994119617202621,"FrontLong":-122.1285687440529,"RearLat":37.994164465226149,"RearLong":-122.12858830974116}
    pedestrian: pedestrian_response['Response'][j]['Json'], dict of pedestrian information, eg:
    {"VehicleId":"188326","VehicleType":"Pedestrian","Longitude":-122.12837294588802,"Latitude":37.993740037830712,"Speed":1.1748109057466443,"Direction":-37.925715339300382,"FrontLat":37.993740630409221,"FrontLong":-122.1283748449418,"RearLat":37.99373903863269,"RearLong":-122.12837327746945}

    return: bool, conflict point: Point
    """
    # transfer gis coordinates to xy coordinates
    ego_front_gis_coord = {
        "latitude": ego_vehicle["FrontLat"],
        "longitude": ego_vehicle["FrontLong"],
    }
    ego_front_xy = gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=ego_front_gis_coord)

    pdst_gis_coord = {
        "latitude": pedestrian["Latitude"],
        "longitude": pedestrian["Longitude"],
    }
    pdst_xy = gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=pdst_gis_coord)

    ego_dir = ego_vehicle["Direction"]
    pdst_dir = pedestrian["Direction"]
    ego_speed = ego_vehicle["Speed"]
    pdst_speed = pedestrian["Speed"]

    ego_speed_x = ego_speed * math.sin(math.radians(ego_dir))
    ego_speed_y = ego_speed * math.cos(math.radians(ego_dir))

    pdst_speed_x = pdst_speed * math.sin(math.radians(pdst_dir))
    pdst_speed_y = pdst_speed * math.cos(math.radians(pdst_dir))

    """
    y=y0+v_y*t
    x=x0+v_x*t
    y=y0+cot(dir)*(x-x0)
    Solve the path equations to see if the two paths will intersact when t>0
    """
    x = (
        (
            (
                pdst_xy.y
                - ego_front_xy.y
                + 1 / math.tan(math.radians(ego_dir)) * ego_front_xy.x
                - 1 / math.tan(math.radians(pdst_dir)) * pdst_xy.x
            )
            / (
                1 / math.tan(math.radians(ego_dir))
                - 1 / math.tan(math.radians(pdst_dir))
            )
        )
        if 1 / math.tan(math.radians(ego_dir)) != 1 / math.tan(math.radians(pdst_dir))
        else None
    )

    if (
        x == None
        or (ego_speed_x > 0 and x < ego_front_xy.x)
        or (ego_speed_x < 0 and x > ego_front_xy.x)
    ):
        return False, None

    if (pdst_speed_x > 0 and x < pdst_xy.x) or (pdst_speed_x < 0 and x > pdst_xy.x):
        return False, None

    y = ego_front_xy.y + 1 / math.tan(math.radians(ego_dir)) * (x - ego_front_xy.x)
    conflict_point = Point(x, y)
    return True, conflict_point


def get_line_equation(x1, y1, x2, y2):
    """
    Calculate the expression of Ax+By+C=0

    return A,B,C
    """
    A = y2 - y1
    B = x1 - x2
    C = x2 * y1 - x1 * y2
    return A, B, C


def CarTrack(ego_json: json):
    """
    Get ego vehicle track equation coefficient.

    Parameter: ego vehicle response, eg:
    {
    "ResponseType": 1,
    "Response": [
        {
            "Json": "{\"Longitude\":-122.12855790836088,\"Latitude\":37.994097773330481,\"Speed\":6.93123831513698,\"Direction\":160.09260222174572,\"Acceleration\":-9.5032290605557268,\"Steer\":-0.001,\"Brake\":0.0,\"FrontLat\":37.994063519246858,\"FrontLong\":-122.12854223150448,\"RearLat\":37.994108129216222,\"RearLong\":-122.1285626478756}",
            "SeqId": 392244992,
            "Time": "2024-05-09T17:33:11.014",
            "Chid": 19200,
            "Flags": 0,
            "Priority": 7,
            "UserId": 0,
            "ChangeLogInfo": null,
            "Events": [
                "40"
            ]
        }
    }
    """
    xy_coords = []

    for entry in ego_json["Response"]:
        data = json.loads(entry["Json"])
        lat_lon = {
            "latitude": data["Latitude"],
            "longitude": data["Longitude"],
        }
        x, y = gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=lat_lon)
        xy_coords.append((x, y))

    line_segments = []

    for i in range(len(xy_coords) - 1):
        x1, y1 = xy_coords[i]
        x2, y2 = xy_coords[i + 1]
        line_eq = get_line_equation(x1, y1, x2, y2)
        line_segments.append(line_eq)

    return line_segments


def PedestrianTrack(pdst_json: json):
    """
    Get pedestrian track equation coefficient

    Parameter: pedestrian response, eg:
    {
    "ResponseType": 1,
    "Response": [
        {
            "Json": "{\"VehicleId\":\"188326\",\"VehicleType\":\"Pedestrian\",\"Longitude\":-122.12837294588802,\"Latitude\":37.993740037830712,\"Speed\":1.1748109057466443,\"Direction\":-37.925715339300382,\"FrontLat\":37.993740630409221,\"FrontLong\":-122.1283748449418,\"RearLat\":37.99373903863269,\"RearLong\":-122.12837327746945}",
            "SeqId": 3949347151,
            "Time": "2024-05-09T17:33:11.375",
            "Chid": 19200,
            "Flags": 0,
            "Priority": 7,
            "UserId": 0,
            "ChangeLogInfo": null,
            "Events": [
                "40"
            ]
        },
        ...
    }
    """
    xy_coords = []
    for entry in pdst_json["Response"]:
        data = json.loads(entry["Json"])
        lat_lon = {
            "latitude": data["Latitude"],
            "longitude": data["Longitude"],
        }
        x, y = gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=lat_lon)
        xy_coords.append((x, y))
    line_segments = []
    for i in range(len(xy_coords) - 1):
        x1, y1 = xy_coords[i]
        x2, y2 = xy_coords[i + 1]
        line_eq = get_line_equation(x1, y1, x2, y2)
        line_segments.append(line_eq)

    return line_segments


def DrawTrack(ego_json: json, pedestrian_json: json):
    xy_coords = []

    for entry in ego_json["Response"]:
        data = json.loads(entry["Json"])
        lat_lon = {
            "latitude": data["Latitude"],
            "longitude": data["Longitude"],
        }
        xy = gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=lat_lon)
        xy_coords.append(xy)

    for i in range(len(xy_coords) - 1):
        p1 = xy_coords[i]
        p2 = xy_coords[i + 1]
        plt.plot([p1.x, p2.x], [p1.y, p2.y], "ro-")

    xy_coords = []

    for entry in pedestrian_json["Response"]:
        data = json.loads(entry["Json"])
        lat_lon = {
            "latitude": data["Latitude"],
            "longitude": data["Longitude"],
        }
        xy = gps_lin_ll2xy(gps_lin=gps_lin, lat_lon=lat_lon)
        xy_coords.append(xy)

    for i in range(len(xy_coords) - 1):
        p1 = xy_coords[i]
        p2 = xy_coords[i + 1]
        plt.plot([p1.x, p2.x], [p1.y, p2.y], "bo-")

    plt.xlabel("X (meters)")
    plt.ylabel("Y (meters)")
    plt.title("Vehicle Trajectory")
    plt.grid(True)
    plt.axis("equal")

    plt.show()
