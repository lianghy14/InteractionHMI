import os
#import hmi_planner
import hmi_planner_safety_index as hmi_planner #B
#import hmi_planner_interaction as hmi_planner #C
from gevent import monkey
monkey.patch_all()
from gevent.queue import Queue
import gevent
import socket
import json
import time

ReceiveQueue = Queue(10)
SendQueue = Queue(5)
durations = []
agents_history = {}
t_hmi_remaining = 0  # set hmi lasting duration (2 second at least)
ehmi = [False, False, False, 0, False, False, False, False, 0, [99, 99, 99, 0.7], [99, 99, 99, 0.7]]
hmi = [False, 0, 0]
outputs_history = [ehmi, hmi, 0]
safety_index_history = []  # record data for robust hmi control
RequestFile = any
ResponseFile = any

def ResetFile():
    global RequestFile
    global ResponseFile
    CloseFile()
    now = time.strftime('%Y_%m_%d_%H_%M_%S')
    RequestFile = open('logs/Request_{0}.txt'.format(now),'w')
    ResponseFile = open('logs/Response_{0}.txt'.format(now),'w')

def CloseFile():
    global RequestFile
    global ResponseFile
    try:
        RequestFile.close()
        ResponseFile.close()
    except:
        pass

def Receiver(UdpServer):
    global RequestFile
    global ResponseFile
    while True:
        try:
            data, addr = UdpServer.recvfrom(2048)
            #print('received message:{0} from PORT {1} on {2}'.format(data, addr[1], addr[0]))
            strdata = data.decode()
            if strdata == 'S':
                ResetFile()
                durations.clear()
                agents_history.clear()
                print('Start')
            elif strdata == 'E':
                CloseFile()
                print('End')
            else:
                ReceiveQueue.put(strdata)
                RequestFile.write(strdata+'\n')
        except:
            print('Receiver Error') 

#{"0" : 0.01 , "1" :["n": "ped", "x": 0.1 ,"y": 0.1]]}
def Consumer():
    global outputs_history
    global safety_index_history
    while True:
        item = ReceiveQueue.get()
        try:
            EHMIRequest = json.loads(item)
            durations.append(EHMIRequest['0'])
            for PawnInfo in EHMIRequest['1']:
                if PawnInfo['n'] not in agents_history:
                    agents_history[PawnInfo['n']] = []
                agents_history[PawnInfo['n']].append([PawnInfo['x'], PawnInfo['y']])
            if len(durations) > 3 and durations[-1] > 12:
                ehmi_info, hmi_info, t_hmi_remaining, safety_index = hmi_planner.generate_hmi_info(agents_history, durations, outputs_history, safety_index_history)
                outputs_history = [ehmi_info, hmi_info, t_hmi_remaining]
                safety_index_history.append(safety_index)
                SendQueue.put([ehmi_info, hmi_info])
        except:
            pass
            #print('Consumer Error : {0}' .format(item)) 

def Sender(UdpClient):
    global ResponseFile
    while True:
        try:
            item = SendQueue.get()
            ehmi_info = item[0]
            EHMIResponse = {'E':[],'T':[],'L':[],'H':[]}
            for i in range(9):
                EHMIResponse['E'].append(int(ehmi_info[i]))
            for i in range(3):
                EHMIResponse['T'].append(ehmi_info[9][i])
            EHMIResponse['T'].append(int(ehmi_info[9][3] * 10))
            for i in range(3):
                EHMIResponse['L'].append(ehmi_info[10][i])
            EHMIResponse['L'].append(int(ehmi_info[10][3] * 10))
            #HMI
            EHMIResponse['H'].append(int(item[1][0]))
            EHMIResponse['H'].append(item[1][1])
            EHMIResponse['H'].append(item[1][2])
            SendData = json.dumps(EHMIResponse, separators=(',',':'))
            UdpClient.sendto(SendData.encode(), ("127.0.0.1" ,6000))
            ResponseFile.write(SendData+'\n')
        except:
            print('Sender Error') 


def main():
    if not os.path.exists('logs'):
        os.mkdir('logs')
    #ResetFile()
    UdpServer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UdpServer.bind(('127.0.0.1', 5000))
    UdpClient = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
    Receiverjob = gevent.spawn(Receiver,UdpServer)
    Consumerjob1 = gevent.spawn(Consumer)
    Consumerjob2 = gevent.spawn(Consumer)
    Senderjob = gevent.spawn(Sender,UdpClient)
    thread_list = [Receiverjob, Consumerjob1,Consumerjob2, Senderjob]
    print('Ready')
    gevent.joinall(thread_list)

    UdpServer.close()
    UdpClient.close()

if __name__ == '__main__':
    main()