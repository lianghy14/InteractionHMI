import redis
import time
import json
import signal
import sys
from genericpath import exists
from ntpath import join
from os import makedirs


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
first_frame_x = 0
first_frame_y = 0


# {”0”:time,”1”:[{”n”:”vehicle1”,”x”:x,”y”:y}, {”n”:”vehicle2”,”x”:x,”y”:y}, {”n”:”vehicle3”,”x”:x,”y”:y}]}

def send_redis_data():
    global frameId
    global demo_data
    global data_length
    line = demo_data[frameId]
    obj = json.dumps(line)
    obj = convert_hmi_data(obj)
    print(obj)
    r.publish('hmi_channel',obj)
    
    if(frameId < (data_length-1)):
        frameId = frameId + 1 
    else:
        frameId = 0
        
def convert_hmi_data(data):
    global first_frame_x
    global first_frame_y
    data = json.loads(data)
    if first_frame_x == 0:
        for vehicle in data["value"]:
            if vehicle["id"] == 0:
                first_frame_x = vehicle["longitude"]
                first_frame_y = vehicle["latitude"]
                break
    result = {
        "0":data["timestamp"],
        "1": [{"n": vehicle["id"], "x": vehicle["longitude"] - first_frame_x, "y": vehicle["latitude"] - first_frame_y} for vehicle in data["value"]]

    }
    return json.dumps(result)


    
        

while True:
    #print("in while loop")
    send_redis_data()

    time.sleep(delay)


