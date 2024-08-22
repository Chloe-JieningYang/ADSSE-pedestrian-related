import math
import sys
from shapely.geometry import Point

"""
class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
"""


class GpsLin:
    def __init__(
        self,
        lat0_deg: float,
        lon0_deg: float,
        lat0_rad: float,
        lon0_rad: float,
        alt0: float,
        radius_ew: float,
        radius_ns: float,
        x: float,
        y: float,
        z: float,
        t: float,
        x_inv: float = 0.0,
        y_inv: float = 0.0,
        t_inv: float = 0.0,
    ):
        self.lat0_deg = lat0_deg
        self.lon0_deg = lon0_deg
        self.lat0_rad = lat0_rad
        self.lon0_rad = lon0_rad
        self.alt0 = alt0
        self.radius_ew = radius_ew
        self.radius_ns = radius_ns
        self.x = x
        self.y = y
        self.z = z
        self.t = t
        self.x_inv = x_inv
        self.y_inv = y_inv
        self.t_inv = t_inv


def radians_to_degrees(radians: float) -> float:
    return radians * 180 / math.pi


def degrees_to_radians(degrees: float) -> float:
    """
    This function converts degrees to radians.
    """
    return degrees * math.pi / 180


def gps_lin_init(gps_lin: GpsLin) -> GpsLin:
    # Convert values to numbers
    for key, value in gps_lin.__dict__.items():
        setattr(gps_lin, key, float(value))

    # Set inv numbers
    s = math.sin(gps_lin.t)
    c = math.cos(gps_lin.t)
    gps_lin.x_inv = -c * gps_lin.x - s * gps_lin.y
    gps_lin.y_inv = s * gps_lin.x - c * gps_lin.y
    gps_lin.t_inv = -gps_lin.t

    return gps_lin


def xyt_transform_xy(gps_lin: GpsLin, xy: Point) -> Point:
    s = math.sin(gps_lin.t_inv)
    c = math.cos(gps_lin.t_inv)
    x = c * xy.x - s * xy.y + gps_lin.x_inv
    y = s * xy.x + c * xy.y + gps_lin.y_inv
    return Point(x, y)


def gps_lin_xy2ll(gps_lin: GpsLin, point: Point) -> dict:
    point = xyt_transform_xy(gps_lin, point)
    dlat_rad = math.asin(point.y / gps_lin.radius_ns)
    dlon_rad = math.asin(point.x / gps_lin.radius_ew / math.cos(gps_lin.lat0_rad))
    latitude = radians_to_degrees(dlat_rad + gps_lin.lat0_rad)
    longitude = radians_to_degrees(dlon_rad + gps_lin.lon0_rad)
    return {"latitude": latitude, "longitude": longitude}


def gps_lin_ll2xy(gps_lin: GpsLin, lat_lon: dict) -> Point:
    """
    This function converts latitude and longitude to x, y in the local coordinate system.
    """
    dlat_rad = degrees_to_radians(lat_lon["latitude"]) - gps_lin.lat0_rad
    dlon_rad = degrees_to_radians(lat_lon["longitude"]) - gps_lin.lon0_rad
    y = math.sin(dlat_rad) * gps_lin.radius_ns
    x = math.sin(dlon_rad) * math.cos(gps_lin.lat0_rad) * gps_lin.radius_ew

    return Point(x, y)


# Create and initialize the GpsLin object fixed for CCTA route
gps_lin = GpsLin(
    lat0_deg=38.00247955319999704,
    lon0_deg=-122.12670135499999446,
    lat0_rad=0.66326839212516353,
    lon0_rad=-2.13151304324457014,
    alt0=37.43156814580000002,
    radius_ew=6386245.37502955179661512,
    radius_ns=6359632.34041758161038160,
    x=0.0,
    y=0.0,
    z=0.0,
    t=0.0,
)
gps_lin = gps_lin_init(gps_lin)

if __name__ == "__main__":
    # Check if two arguments are provided
    if len(sys.argv) != 3:
        print("Usage: python script.py <x_value> <y_value>")
        sys.exit(1)

    try:
        # Parse the command-line arguments
        x_value = float(sys.argv[1])
        y_value = float(sys.argv[2])
    except ValueError as e:
        print(f"Error parsing arguments: {e}")
        sys.exit(1)

    # Create a Point object with x, y values
    point = Point(x_value, y_value)

    # Transform the xy point to latitude and longitude
    lat_long = gps_lin_xy2ll(gps_lin, point)

    # Print the result
    print("Latitude:", lat_long["latitude"])
    print("Longitude:", lat_long["longitude"])

# python3 convert_xy_to_gis.py -384.3115471328425, -275.4964434083863
