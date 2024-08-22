import json
import time
import requests
from datetime import datetime

# Endpoint
get_history_data_API = "http://70.229.15.100:9080/txvapi/history/getHistoryData"


def FetchEgoVehicleData(start_time, end_time):
    """
    The function to get ego-vehicle data

    parameters: start time: eg. 2024-05-06 21:43:54
                end time: eg. same above
                 Notice: the time period should not be too long, otherwise the response will be erroneous

    return:     json
    """

    params = dict()
    params["DataType"] = "MotionData"
    params["FilterFlags"] = 2
    """
    Bit 1: Exclusive event search - return only rows which belongs to all of the events specified in EventFilterList.
    Bit 2: Fetch events - retrieves EventId column (automatic join with event table) for each row
    """
    time_filter = {
        "StartTime": start_time.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "EndTime": end_time.strftime("%Y-%m-%dT%H:%M:%SZ"),
    }

    params["TimeFilter"] = time_filter
    response = requests.post(get_history_data_API, json=params)
    # check status
    if response.status_code == 200:
        data = response.json()
    else:
        print("fail to fetch, status code: {response.status_code}")
        data = None

    return data


def FetchNeighbourData(start_time, end_time):
    """
    The function to get neighbour data

    parameters: start time: eg. 2024-05-06 21:43:54
                end time: eg. same above
                 Notice: the time period should not be too long, otherwise the response will be erroneous

    return:     json
    """

    params = dict()
    params["DataType"] = "MotionDetectionData"
    params["FilterFlags"] = 2
    """
    Bit 1: Exclusive event search - return only rows which belongs to all of the events specified in EventFilterList.
    Bit 2: Fetch events - retrieves EventId column (automatic join with event table) for each row
    """
    time_filter = {
        "StartTime": start_time.strftime("%Y-%m-%dT%H:%M:%SZ"),
        "EndTime": end_time.strftime("%Y-%m-%dT%H:%M:%SZ"),
    }

    filter_obj = {
        "PropertyName": "VehicleType",
        "Operator": "=",
        "FilterValues": ["Pedestrian"],
    }

    params["TimeFilter"] = time_filter
    params["Filters"] = [[filter_obj]]
    response = requests.post(get_history_data_API, json=params)
    # check status
    if response.status_code == 200:
        data = response.json()
    else:
        print("fail to fetch, status code: {response.status_code}")
        data = None

    return data


def GetCorrespondingEgoData(ego_vehicle_dataset: json, pedestrian_data: dict) -> dict:
    """
    Get ego vehicle data of the same time as pedestrian data.

    Parameter: json reponse of ego vehicle data, pedestrian data of a certain time

    Return: ego vehicle data dict or None
    """
    for data_piece in ego_vehicle_dataset["Response"]:
        if data_piece["Time"] == pedestrian_data["Time"]:
            return data_piece
    return None


'''
"""
Notice: the time period should not be too long, otherwise the response will be erroneous
"""
start_time_str = "2024-05-09 17:33:00"
end_time_str = "2024-05-09 17:34:00"

start_time = datetime.strptime(start_time_str, "%Y-%m-%d %H:%M:%S")
end_time = datetime.strptime(end_time_str, "%Y-%m-%d %H:%M:%S")

result = FetchNeighbourData(start_time, end_time)
# result = FetchEgoVehicleData(start_time, end_time)

if result:
    print("successful")
    print(result)
else:
    print("unsuccessful")
'''
