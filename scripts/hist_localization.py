# from rospy_message_converter import json_message_converter
import rospy
from sensor_msgs.msg import LaserScan
from mymath import *
import signal
import time
import json
import pickle
import numpy as np
import matplotlib.pyplot as plt
###################################
# def twoPointsDist(a,b):
#     return math.sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))

def handler(signum, frame):
    res = input("\n\nCtrl-c was pressed. Do you really want to exit? y/n ")
    if res == 'y':
        exit(1)


signal.signal(signal.SIGINT, handler)


class HistogramLocalization():

    __map = []
    __orientMap = []
    __rawdata = []
    __scan = []
    
    def __init__(self, nodeName="Histogram_Localization",bins=20, orientSection=20):
        rospy.init_node(nodeName)
        self.__binsNumber=bins
        self.__orientSectionNumber=orientSection

    def loadMapJson(self, fileName):
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

    def loadMapPickle(self, fileName):
        _scan = []
        _scango = []
        _pose = [0, 0, 0]
        #json_data = open(fileName)
        with open(fileName, 'rb') as f:
            data = pickle.load(f)
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

    def MakeHistMap(self):
        for i in self.__rawdata:
            # print(len(i[1]))
            hist, binno = np.histogram(
                i[1], range=(0.0, 10.0), bins=self.__binsNumber)
            self.__map.append((i[0], hist, binno, self.__binsNumber, (0.0, 10.0)))

    def MakeOrientMap(self):
        for scan in self.__rawdata:
            tmpOrientList = []
            for j in range(0, self.__orientSectionNumber):
                sum = 0
                tmpOrientList = []
                for i in range(0, len(scan[1])//self.__orientSectionNumber):
                    if (
                        not math.isnan(scan[1][i + self.__orientSectionNumber*j])
                        and not math.isinf(scan[1][i + self.__orientSectionNumber*j])
                    ):
                        sum += scan[1][i + self.__orientSectionNumber*j]
                tmpOrientList.append(sum/(len(scan[1])//self.__orientSectionNumber))

            self.__orientMap.append(tmpOrientList)
        # print(self.__orientMap)
###################################################

    def generateOrientList(self, scan):
        orientList = []
        for j in range(0, self.__orientSectionNumber):
            sum = 0
            counter = 0
            for i in range(0, (len(scan)//self.__orientSectionNumber)):
                if (
                    not math.isnan(scan[i + self.__orientSectionNumber*j])
                    and not math.isinf(scan[i + self.__orientSectionNumber*j])
                ):
                    counter += 1
                    sum += scan[i + self.__orientSectionNumber*j]
            if counter != 0:
                orientList.append(sum/(counter))
            else:
                orientList.append(0)

        return orientList
#######################################################

    def locateRobot(self):
        print(len(self.__scan))
        tmpscan = self.__scan
        hist1, b = np.histogram(tmpscan, range=(0.0, 10.0), bins=self.__binsNumber)
        currentOrientList = self.generateOrientList(tmpscan)
        diffList = []
        for i in self.__map:
            diff = 0
            for j in range(0, self.__binsNumber):
                diff += abs(i[1][j]-hist1[j])
            diffList.append(diff)
        diffList = np.array(diffList)
        print('Selected point:', self.__map[np.argmin(diffList)][0])
        # orientation
        tmpOrient = self.__orientMap[np.argmin(diffList)].copy()
        copyCurrentList = currentOrientList.copy()
        diffOrient = []

        sum = 0
        for i in range(0, len(tmpOrient)):
            sum += abs(tmpOrient[i]-copyCurrentList[i])
        diffOrient.append(sum)
        copyCurrentList.append(copyCurrentList.pop(0))

        while currentOrientList != copyCurrentList:
            sum = 0
            for i in range(0, len(copyCurrentList)):
                sum += abs(tmpOrient[i]-copyCurrentList[i])
            diffOrient.append(sum)
            copyCurrentList.append(copyCurrentList.pop(0))
        orient = math.radians(abs(np.argmin(diffOrient))*(360//self.__binsNumber))
        print("Orientation: ", orient)
        return {"point":self.__map[np.argmin(diffList)][0],"orientation":orient}

    def locateRobotScan(self,scanT):
        #print(len(self.__scan))
        tmpscan = scanT
        hist1, b = np.histogram(tmpscan, range=(0.0, 10.0), bins=self.__binsNumber)
        currentOrientList = self.generateOrientList(tmpscan)
        diffList = []
        for i in self.__map:
            diff = 0
            for j in range(0, self.__binsNumber):
                diff += abs(i[1][j]-hist1[j])
            diffList.append(diff)
        diffList = np.array(diffList)
        # print('Selected point:', self.__map[np.argmin(diffList)][0])
        # orientation
        tmpOrient = self.__orientMap[np.argmin(diffList)].copy()
        copyCurrentList = currentOrientList.copy()
        diffOrient = []

        sum = 0
        for i in range(0, len(tmpOrient)):
            sum += abs(tmpOrient[i]-copyCurrentList[i])
        diffOrient.append(sum)
        copyCurrentList.append(copyCurrentList.pop(0))

        while currentOrientList != copyCurrentList:
            sum = 0
            for i in range(0, len(copyCurrentList)):
                sum += abs(tmpOrient[i]-copyCurrentList[i])
            diffOrient.append(sum)
            copyCurrentList.append(copyCurrentList.pop(0))
        orient = math.radians(abs(np.argmin(diffOrient))*(360//self.__binsNumber))
        # print("Orientation: ", orient)
        return {"point":self.__map[np.argmin(diffList)][0],"orientation":orient}
    
    
    def callback(self, msg):
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

    def initTopicConnection(self, topicName='/laser/scan'):
        rospy.Subscriber(topicName, LaserScan, self.callback)

    def exitTopicConnection(self):
        self.sub.unregister()


# def __main__():
#     # fileName = './map/testMap.json'
#     # hm = HistogramLocalization()
#     # hm.initTopicConnection()
#     # time.sleep(0.1)
#     # hm.loadMap(fileName)
#     # # hm.MakeHistMap()
#     # # hm.MakeOrientMap()
#     # # hm.printHistMap()

#     # print('###############################\n')
#     # hm.locateRobot()
#     # print('\n###############################')
#     # # hm.printHistOrientMap()
#     # rospy.spin()

# if __name__ == "__main__":
#     __main__()
