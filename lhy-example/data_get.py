import os
import json
import redis
import gevent
from gevent.queue import Queue
import hmi_planner_safety_index as hmi_planner  # B
import time
from ntpath import join
from os import makedirs, path, remove

# 连接到Redis服务器
host = "202.120.189.56"
port = 5010
password = 'Wanji@300552!'
now = time.strftime('%Y_%m_%d_%H_%M_%S')

# HMI算法预定义变量
ReceiveQueue = Queue(10)
SendQueue = Queue(5)
durations = []
agents_history = {}
t_hmi_remaining = 0  # set hmi lasting duration (2 second at least)
ehmi = [False, False, False, 0, False, False, False, False, 0, [99, 99, 99, 0.7], [99, 99, 99, 0.7]]
hmi = [False, 0, 0]
outputs_history = [ehmi, hmi, 0]
safety_index_history = []  # record data for robust hmi control

# 算法计算
def calculate_hmi(data):
    global outputs_history
    global safety_index_history
    global hmi
    # try:
    if data:
        HMIRequest = data
        durations.append(HMIRequest['0'])
        for PawnInfo in HMIRequest['1']:
            if PawnInfo['n'] not in agents_history:
                agents_history[PawnInfo['n']] = []
            agents_history[PawnInfo['n']].append([PawnInfo['x'], PawnInfo['y']])
        if len(durations) > 3 and durations[-1] > 12:
            ehmi_info, hmi_info, t_hmi_remaining, safety_index = hmi_planner.generate_hmi_info(
                agents_history, durations, outputs_history, safety_index_history
            )
            outputs_history = [ehmi_info, hmi_info, t_hmi_remaining]
            safety_index_history.append(safety_index)
            SendQueue.put(hmi_info)
        else:
            SendQueue.put(hmi)
    # except Exception as e:
    #     print(f"Error in calculate_hmi: {e}")


# 输出HMI控制指令与logs存储
def output_hmi_command(data):
    global now
    try:
        item = SendQueue.get()  # Introduce a timeout of 1 second
        hmi_info = item[0]  # Assuming hmi_info is the first element of the item
        HMIResponse = {'H': []}
        # HMI
        HMIResponse['H'].append(int(hmi_info))  # Use hmi_info directly
        HMIResponse['H'].append(item[1])
        HMIResponse['H'].append(item[2])
        SendData = json.dumps(HMIResponse, separators=(',', ':'))

        # Write to response.txt
        ResponseFile = open('logs/Response_{0}.txt'.format(now), 'a')
        with ResponseFile as f:  # Open in append mode
            f.write(SendData + "\n")

        print('Out data '+SendData + '\n')  # 打印算法结果到窗口
        # Publish to Redis
        r.publish('hmi_channel_output', SendData)

        RequestData = json.dumps(data, separators=(',', ':'))
        # Write to request.txt
        RequestFile = open('logs/Request_{0}.txt'.format(now), 'a')
        with RequestFile as f:  # Open in append mode
            f.write(RequestData + "\n")
    except Exception as e:
        print(f'Sender Error: {e}')


# 获取实时数据
def get_sub_data():
    sub = r.pubsub()
    sub.subscribe('hmi_channel')
    for msg in sub.listen():
        if msg['type'] == 'message':
            print("Get data", msg['data'].decode('utf-8'))
            data = json.loads(msg['data'].decode('utf-8'))
            # calculate logic
            calculate_hmi(data)  # Call the calculation function after receiving data
            output_hmi_command(data)  # Call the output function after calculation
            # r.publish('hmi_channel_output', escaped)


# 读取实时数据
r = redis.StrictRedis(host=host, port=port, password=password, db=0)
# r = redis.StrictRedis(host='127.0.0.1', port=6379, db=0)
get_sub_data()