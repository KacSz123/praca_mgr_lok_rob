#from rospy_message_converter import json_message_converter
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from mymath import *
import signal
import time
import json
import numpy as np
import matplotlib.pyplot as plt
###################################
def handler(signum, frame):
    res = input("\n\nCtrl-c was pressed. Do you really want to exit? y/n ")
    if res == 'y':
        exit(1)

signal.signal(signal.SIGINT, handler)

class HistMap():
    
    __map = []
    __orientMap = []
    __rawdata = []
    __scan = []
    
    def __init__(self, nodeName="myNodeHistLocalization"):
        rospy.init_node(nodeName)
    def loadMap(self,fileName):
        _scan = []
        _scango = []
        _pose = [0, 0, 0]
        json_data = open(fileName)
        data = json.load(json_data)
        _scan = []
        for j in range(0, len(data)):
            # Wczytanie skanu z pierwszego zestawu danych
            _scan = []
            _scango = data[j]["scan"]
            _pose = data[j]["pose"]
            for i in range(0, len(_scango)):
                if (
                    not math.isnan(_scango[i])
                    and not math.isinf(_scango[i])
                ):
                    _scan.append(_scango[i])
            self.__rawdata.append((_pose, _scan))
            self.__orientMap.append(self.generateOrientList(_scango))
        self.MakeHistMap()


    
    def MakeHistMap(self, binsNumber=20):
        for i in self.__rawdata:
            #print(len(i[1]))
            hist, binno = np.histogram(i[1], range=(0.0, 10.0), bins=binsNumber)
            self.__map.append((i[0], hist, binno, binsNumber, (0.0, 10.0)))
            
    def MakeOrientMap(self, b=20):
        for scan in self.__rawdata:
            tmpOrientList=[]  
            for j in range(0,b):
                sum = 0
                tmpOrientList=[]    
                for i in range(0, len(scan[1])//b):
                        if (
                            not math.isnan(scan[1][i + b*j])
                            and not math.isinf(scan[1][i + b*j])
                            
                        ):
                            sum+=scan[1][i+ b*j]
                tmpOrientList.append(sum/(len(scan[1])//b))
            
            self.__orientMap.append(tmpOrientList)
        #print(self.__orientMap) 
            
    def generateOrientList(self,scan,b=20):
        orientList = []
        for j in range(0,b):
            sum = 0
            counter = 0    
            for i in range(0, len(scan)//b):
                    if (
                        not math.isnan(scan[len(scan)//b + b*j])
                        and not math.isinf(scan[len(scan)//b + b*j])
                    ):
                        counter+=1
                        sum+=scan[len(scan)//b + b*j]
            print(counter)
            orientList.append(sum/(counter))
        print(len(orientList))
        return orientList
    
    def locateRobot(self):
        print(len(self.__orientMap ))
        tmpscan = self.__scan
        hist1, b = np.histogram(tmpscan, range=(0.0, 10.0), bins=20)
        currentOrientList= self.generateOrientList(tmpscan)
        diffList = []
        for i in self.__map:
            diff = 0
            for j in range(0, 20):
                diff += abs(i[1][j]-hist1[j])
            diffList.append(diff)
        diffList = np.array(diffList)
        print('Przyblizone polozenie to:', self.__map[np.argmin(diffList)][0])
        
        
        
        copyCurrentList = currentOrientList.copy()
        tmpOrient = self.__orientMap[np.argmin(diffList)]
        diffOrient=[]
        sum = 0
        for i in range(0,len(tmpOrient)):
                sum -= abs(tmpOrient[i]-copyCurrentList[i])

        diffOrient.append(sum)
        copyCurrentList = copyCurrentList[-1:]+copyCurrentList[:-1]
        
        # copyCurrentList = copyCurrentList[-1:]+copyCurrentList[:-1]
        # print("!!!!!!!!!! ########### !!!!!!!!!!!!!!")
        # print(len(currentOrientList))
        # print("!!!!!!!!!! ########### !!!!!!!!!!!!!!")
        # print(len(tmpOrient))
        # print("!!!!!!!!!! ########### !!!!!!!!!!!!!!")
        while currentOrientList != copyCurrentList:
            sum = 0
            for i in range(0,len(tmpOrient)):
                sum -= abs(tmpOrient[i]-copyCurrentList[i])

            diffOrient.append(sum)
            copyCurrentList = copyCurrentList[-1:]+copyCurrentList[:-1]
        print(len(diffOrient))
        orient = 0
        if math.radians(abs(np.argmin(diffOrient))*18) <=math.pi:
            orient = math.radians(abs(np.argmin(diffOrient))*18)+math.pi
        else:
            orient = math.radians(abs(np.argmin(diffOrient))*18)-math.pi
        print("Orientacja!!!!!: ", orient)

    def callback(self,msg):
        self.__scan = msg.ranges
    def printData(self):
        for i in self.__rawdata:
            print(i)
    def printHistMap(self):
        for i in self.__map:
            print(i)
            print('\n')
    def printHistOrientMap(self):
        for i in self.__orientMap:
            print(i)
            print('\n')
      
    
    def initTopicConnection(self, topicName='/m2wr/laser/scan'):
        rospy.Subscriber(topicName, LaserScan, self.callback)
        
            
    def exitTopicConnection(self):
        self.sub.unregister()
            
            


def __main__():
    fileName = 'map11.json'
    hm = HistMap()
    hm.initTopicConnection()
    time.sleep(0.1)
    hm.loadMap(fileName)
    #hm.MakeHistMap()
    # hm.MakeOrientMap()
    # hm.printHistMap()
    hm.locateRobot()
    
    print('###############################')
    # hm.printHistOrientMap()

if __name__ == "__main__":
    __main__()
