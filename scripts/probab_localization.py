# from rospy_message_converter import jsonsectionsNumber_message_converter
import rospy
from sensor_msgs.msg import LaserScan

import numpy as np
import json
import pickle

from errors_and_results import LocalizationResult, ProbabLocalizationErrors
from mymath import *

###################################




class ProbabLocalization():
    def __init__(self, sectionsNumberLocalization=8, sectionsNumberOrient: int=8, mRes: int=0.1):
        self.__posMap = []
        self.__orientMap = []
        self.__rawdata = []
        self.__scan = []
        self.__sectionsNumberLocalization = sectionsNumberLocalization
        self.__sectionNumberOrient = sectionsNumberOrient
        self.__mapRes = round(mRes, 2)

    def loadMapOrientJson(self, fileName: str):
        _scango = []
        _pose = [0, 0, 0]
        json_data = open(fileName)
        data = json.load(json_data)
        for j in range(0, len(data)):
            # Wczytanie skanu z pierwszego zestawu danych
            _scango = data[j]["scan"]
            _pose = data[j]["pose"]

            self.__orientMap.append(
                (_pose, self.makeProbabDescrOneScanOrient(_scango)))
        # print(self.__orientMap)

    def loadMapOrientPickle(self, fileName: str):
        _scango = []
        _pose = [0, 0, 0]

        with open(fileName, 'rb') as f:
            data = pickle.load(f)
        for j in range(0, len(data)):
            # Wczytanie skanu z pierwszego zestawu danych
            _scango = data[j]["scan"]
            _pose = data[j]["pose"]

            self.__orientMap.append(
                (_pose, self.makeProbabDescrOneScanOrient(_scango)))


    def loadMapLocalMin(self, fileName):
        self.loadMapOrientJson(fileName)
        _scango = []
        _pose = [0, 0, 0]
        json_data = open(fileName)
        data = json.load(json_data)
        for j in range(0, len(data)):
            # Wczytanie skanu z pierwszego zestawu danych
            _scango = data[j]["scan"]
            _pose = data[j]["pose"]
            _pose = [round(_pose[i], 2) for i in range(0, 3)]
            self.__posMap.append(
                (_pose, self.makeProbabDescrOneScanLocalMin(_scango)))
        # print(self.__posMap)

    def loadMapLocalMinPickle(self, fileName):
        self.loadMapOrientPickle(fileName)
        _scango = []
        _pose = []
        with open(fileName, 'rb') as f:
            data = pickle.load(f)
        for j in range(0, len(data)):
            # Wczytanie skanu z pierwszego zestawu danych
            _scango = list(data[j]["scan"])
            _pose = data[j]["pose"]
            _pose = [round(_pose[i], 2) for i in range(0, 3)]

            self.__posMap.append(
                (_pose, self.makeProbabDescrOneScanLocalMin(_scango)))

    def makeProbabDescrOneScanOrient(self, scan):
        perSection = (len(scan))//self.__sectionNumberOrient
        currentProbList = []
        for i in range(0, self.__sectionNumberOrient):
            currentList = []

            for j in range(0, perSection):
                if (
                    not math.isnan(scan[i*perSection+j])
                    and not math.isinf(scan[i*perSection+j])
                ):
                    currentList.append(scan[i*perSection+j])
            if len(currentList) != 0:
                currentProbList.append((GetMuSigmaFromEqSqrt(currentList)))
            else:
                currentProbList.append((0.0, 0.0))
        return currentProbList

    def makeProbabDescrOneScanLocalMin(self, scan):
        localMin = np.argmin(scan)
        newScan = scan[localMin:-1] + [scan[-1]]+scan[0:localMin]
        perSection = (len(scan))//self.__sectionsNumberLocalization
        currentProbList = []

        for i in range(0, self.__sectionsNumberLocalization):
            currentList = []

            for j in range(0, perSection):
                if (
                    not math.isnan(newScan[i*perSection+j])
                    and not math.isinf(newScan[i*perSection+j])
                ):
                    currentList.append(newScan[i*perSection+j])
            currentProbList.append((GetMuSigmaFromEqSqrt(currentList)))
        return currentProbList

    def __MarkvectSubstract(self, a, b):
        diff = 0
        for j in range(0, self.__sectionsNumberLocalization):
            diff += twoPointsDist(a[j], b[j])
        return diff

    def __MarkvectMean(self, a, b):
        diff = 0
        for j in range(0, self.__sectionsNumberLocalization):
            diff += twoPointsDist(a[j], b[j])
        return diff/self.__sectionsNumberLocalization

    def locateRobotLocalMinimum(self, fileScan=None, G=(1.05, 1.05)) -> LocalizationResult:
        tmpscan = []
        if fileScan == None:
            tmpscan = self.__scan
        elif fileScan != None:
            tmpscan = list(fileScan)
        # print(isinstance(tmpscan,list))
        tmpProbList = self.makeProbabDescrOneScanLocalMin(list(tmpscan))
        tmpOrientList = self.makeProbabDescrOneScanOrient(list(tmpscan))
        diffList = []
        for i in self.__posMap:
            diff = 0
            for j in range(0, self.__sectionsNumberLocalization):
                diff += twoPointsDist(i[1][j], tmpProbList[j])
            diffList.append(diff)

        # orientation
        currentPoint = self.__orientMap[np.argmin(diffList)][1]
        copyCurrentPoint = currentPoint.copy()
        sum = 0
        diffOrient = []
        for i in range(0, len(tmpOrientList)):
            sum += abs((tmpOrientList[i][0]-currentPoint[i]
                       [0])+(tmpOrientList[i][1]-currentPoint[i][1]))
        diffOrient.append(sum)
        currentPoint.append(currentPoint.pop(0))

        sum = 0
        while copyCurrentPoint != currentPoint:
            sum = 0
            for i in range(0, len(tmpOrientList)):
                sum += abs((tmpOrientList[i][0]-currentPoint[i]
                           [0])+(tmpOrientList[i][1]-currentPoint[i][1]))
            diffOrient.append(sum)
            currentPoint.append(currentPoint.pop(0))
        orient = 0
        # print(self.__posMap[np.argmin(diffList)][0], ' poczatkowy')
        orient = math.radians(np.argmin(diffOrient) *
                              (360//self.__sectionNumberOrient))

        return LocalizationResult(self.__map[np.argmin(diffList)][0], orient)

        # print(currentPoint)
    def locateRobotLocalMinimumNeighbours(self, fileScan=None, G=(1.05, 1.05)):
        tmpscan = []
        if fileScan == None:
            tmpscan = self.__scan
        elif fileScan != None:
            tmpscan = list(fileScan)
        # print(isinstance(tmpscan,list))
        tmpProbList = self.makeProbabDescrOneScanLocalMin(list(tmpscan))
        tmpOrientList = self.makeProbabDescrOneScanOrient(list(tmpscan))
        diffList = []
        for i in self.__posMap:
            diff = 0
            for j in range(0, self.__sectionsNumberLocalization):
                diff += twoPointsDist(i[1][j], tmpProbList[j])
            diffList.append(diff)
        # print("Selected point:", self.__posMap[np.argmin(diffList)][0])

        a = self.__posMap[np.argmin(diffList)][0]
        Apoint = self.__posMap[np.argmin(diffList)][1]
        # Apoint = self.__posMap[np.argmin(diffList)][0]
        searchListX = [[round(a[0]+self.__mapRes, 1), round(a[1], 1), 0],
                       [round(a[0]-self.__mapRes, 1), round(a[1], 1), 0]]
        searchListY = [[round(a[0], 1), round(
            a[1]+self.__mapRes, 1), 0], [round(a[0], 1), round(a[1]-self.__mapRes, 1), 0]]
       # sasiedzi
        pointsXList = []
        neighbXErrList = []
        pointsYList = []
        neighbYErrList = []
        xProb = []
        yProb = []

        for j in self.__posMap:
            for i in searchListX:
                if i[0] == j[0][0] and i[1] == j[0][1]:
                    pointsXList.append(j[0])
                    xProb.append(j[1])
                    neighbXErrList.append(
                        self.__MarkvectSubstract(j[1], tmpProbList))
            for i in searchListY:
                if i[0] == j[0][0] and i[1] == j[0][1]:
                    pointsYList.append(j[0])
                    neighbYErrList.append(
                        self.__MarkvectSubstract(j[1], tmpProbList))
                    yProb.append(j[1])
        neighbXErrList = np.array(neighbXErrList)
        aX = np.argmin(neighbXErrList)
        neighbYErrList = np.array(neighbYErrList)
        aY = np.argmin(neighbYErrList)
        # orientation
        currentPoint = self.__orientMap[np.argmin(diffList)][1]
        copyCurrentPoint = currentPoint.copy()
        sum = 0
        diffOrient = []
        for i in range(0, len(tmpOrientList)):
            sum += abs((tmpOrientList[i][0]-currentPoint[i]
                       [0])+(tmpOrientList[i][1]-currentPoint[i][1]))
        diffOrient.append(sum)
        currentPoint.append(currentPoint.pop(0))

        sum = 0
        while copyCurrentPoint != currentPoint:
            sum = 0
            for i in range(0, len(tmpOrientList)):
                sum += abs((tmpOrientList[i][0]-currentPoint[i]
                           [0])+(tmpOrientList[i][1]-currentPoint[i][1]))
            diffOrient.append(sum)
            currentPoint.append(currentPoint.pop(0))
        orient = 0
        # print(self.__posMap[np.argmin(diffList)][0], ' poczatkowy')
        orient = math.radians(np.argmin(diffOrient) *
                              (360//self.__sectionNumberOrient))

        zX = self.__MarkvectSubstract(Apoint, xProb[aX])
        zY = self.__MarkvectSubstract(Apoint,  yProb[aY])

        xiX = abs(1-(neighbXErrList[aX]/zX))*G[0]
        xiY = abs(1-(neighbYErrList[aY]/zY))*G[1]

        p = [round(self.__posMap[np.argmin(diffList)][0][0]+(pointsXList[aX][0] - self.__posMap[np.argmin(diffList)][0][0])*xiX, 4),
             round(self.__posMap[np.argmin(diffList)][0][1]+(pointsYList[aY][1] - self.__posMap[np.argmin(diffList)][0][1])*xiY, 4)]

        return {"point": p, "orientation": orient, "ksiX": xiX, "ksiY": xiY}

        # print(currentPoint)


######################################################################


    def callback(self, msg):
        self.__scan = msg.ranges

    def printData(self):
        for i in self.__rawdata:
            print(i)

    def initTopicConnection(self, topicName='/laser/scan', _nodeName="Probabilistic_Localization",):
        rospy.init_node(nodeName=_nodeName)
        rospy.Subscriber(topicName, LaserScan, self.callback)

    def exitTopicConnection(self):
        self.sub.unregister()
