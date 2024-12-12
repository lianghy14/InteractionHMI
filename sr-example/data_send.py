import redis
import time
import json
import signal
import sys
from genericpath import exists
from ntpath import join
from os import makedirs
import pyproj
import numpy as np


# 连接到Redis服务器
host = "202.120.189.56"
port = 5010
password='Wanji@300552!'
        
# 连接到Redis服务器
#r = redis.StrictRedis(host='127.0.0.1', port=6379, db=0)
r = redis.StrictRedis(host=host, port=port, password=password,db=0)

frequency = 20
delay = 2/frequency

demo_data = open('mock_sr_data.csv','r').readlines()


data_length = len(demo_data)


frameId = 0 
mqtt_frameId = 0
first_line = demo_data[0]
first_line = json.loads(first_line)
first_frame_x = first_line["value"][0]["longitude"]
first_frame_y = first_line["value"][0]["latitude"]

# {”0”:time,”1”:[{”n”:”vehicle1”,”x”:x,”y”:y}, {”n”:”vehicle2”,”x”:x,”y”:y}, {”n”:”vehicle3”,”x”:x,”y”:y}]}
# {"0":0.0803152,"1":[{"n":"ego","x":340579.46668973094,"y":434948.58075410436},{"n":"ped","x":331866.2934785306,"y":423449.29921511223}]}

def send_redis_data():
    global frameId
    global demo_data
    global data_length
    global first_frame_x
    global first_frame_y

    line = demo_data[frameId]
    obj = line#json.dumps(line)
    obj = convert_data_to_cartesian(json.loads(obj), first_frame_x, first_frame_y)
    print(obj)
    r.publish('hmi_channel',obj)
    
    if(frameId < (data_length-1)):
        frameId = frameId + 1 
    else:
        frameId = 0
        
        
def convert_data_to_cartesian(data, origin_lon, origin_lat, origin_alt=0.0):
    """
    将原始数据转换为目标数据格式，并进行坐标系转换。

    参数：
    - data (dict): 原始数据字典。
    - origin_lon (float): 原点经度。
    - origin_lat (float): 原点纬度。
    - origin_alt (float): 原度高度，默认为0.0米。

    返回：
    - str: 目标数据格式的 JSON 字符串。
    """
    # 转换函数，用于将经纬度转换为 ECEF 坐标
    def geodetic_to_ecef(lon, lat, alt, a=6378137.0, f=1/298.257223563):
        lon_rad = np.deg2rad(lon)
        lat_rad = np.deg2rad(lat)
        e_sq = f * (2 - f)
        N = a / np.sqrt(1 - e_sq * np.sin(lat_rad)**2)
        x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
        z = (N * (1 - e_sq) + alt) * np.sin(lat_rad)
        return (x, y, z)
    
    # 转换函数，用于将 ECEF 坐标转换为 ENU 坐标
    def ecef_to_enu(x, y, z, origin_x, origin_y, origin_z, lon_origin, lat_origin):
        lon_origin_rad = np.deg2rad(lon_origin)
        lat_origin_rad = np.deg2rad(lat_origin)
        
        sin_lat = np.sin(lat_origin_rad)
        cos_lat = np.cos(lat_origin_rad)
        sin_lon = np.sin(lon_origin_rad)
        cos_lon = np.cos(lon_origin_rad)
        
        # ENU 到 ECEF 的旋转矩阵
        R = np.array([
            [-sin_lon, cos_lon, 0],
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]
        ])
        
        delta = np.array([x - origin_x, y - origin_y, z - origin_z])
        enu = R.dot(delta)
        return enu
    
    # 初始化车辆列表
    vehicles = []
    ego = None
    ped_count = 0
    veh_count = 0
    
    # 提取时间戳
    timestamp = data['timestamp']
    
    # 提取原点的 ECEF 坐标
    origin_x, origin_y, origin_z = geodetic_to_ecef(origin_lon, origin_lat, origin_alt)
    
    # 遍历每辆车
    for vehicle in data['value']:
        id_ = vehicle['id']
        lon = vehicle['longitude']
        lat = vehicle['latitude']
        vehicle_type = vehicle['vehicleType']
        
        # 转换 ECEF
        x_ecef, y_ecef, z_ecef = geodetic_to_ecef(lon, lat, 0.0)
        
        # 转换 ENU
        enu = ecef_to_enu(x_ecef, y_ecef, z_ecef, origin_x, origin_y, origin_z, origin_lon, origin_lat)
        x = enu[0]
        y = enu[1]
        
        # 根据类型命名
        if id_ == 0:
            n = "ego"
            ego = {"n": n, "x": x, "y": y}
        elif vehicle_type == 2:
            ped_count += 1
            n = f"ped{ped_count}"
            vehicles.append({"n": n, "x": x, "y": y})
        else:
            veh_count += 1
            n = f"veh{veh_count}"
            vehicles.append({"n": n, "x": x, "y": y})
    
    # 构建目标数据
    target_data = {
        "0": float(timestamp),
        "1": vehicles
    }
    
    if ego:
        target_data['1'].insert(0, ego)
    
    return json.dumps(target_data, ensure_ascii=False)

    
        

while True:
    #print("in while loop")
    send_redis_data()

    time.sleep(delay)


