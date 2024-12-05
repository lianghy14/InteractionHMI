import json
import redis
import time
from ntpath import join
from os import makedirs,path,remove

# 连接到Redis服务器
host = "202.120.189.56"
port = 5010
password='Wanji@300552!'


r = redis.StrictRedis(host=host, port=port, password=password,db=0)
#r = redis.StrictRedis(host='127.0.0.1', port=6379 ,db=0)
            
# 获取实时数据
def get_sub_data():
    sub = r.pubsub()
    sub.subscribe('hmi_channel')
    for msg in sub.listen():
        if msg['type'] == 'message':
            print("Get data",msg['data'].decode('utf-8'))
            data = json.loads(msg['data'].decode('utf-8'))
            compressed = json.dumps(data, separators=(',', ':'))
            escaped = compressed.replace('\n', '\\n')
            print(escaped) 
            # calculate logic
            
            r.publish('hmi_channel_output',escaped)      


#get_realtime_data()

get_sub_data()
