# from rospy_message_converter import json_message_converter
import rospy
from sensor_msgs.msg import LaserScan

import json
import pickle
import numpy as np

from errors_and_results import LocalizationResult, HistLocalizationErrors
from mymath import *

###################################
# def twoPointsDist(a,b):
#     return math.sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))



class HistogramLocalization():

    
    def __init__(self, nodeName="Histogram_Localization",bins=20, orientSection=20, mRes=0.1):
        self.__map = []
        self.__orientMap = []
        self.__rawdata = []
        self.__scan = []
        self.__ifSubscribing=False
        self.__nodeName = nodeName
        self.__binsNumber=bins
        self.__orientSectionNumber=orientSection
        self.__mapRes = round(mRes,2)
        
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
            self.__orientMap.append(self.generateOrientList(list(_scango)))
        self.makeHistMap()

    def loadMapPickle(self, fileName):
        _scan = []
        _scango = []
        _pose = [0, 0, 0]
        #json_data = open(fileName)
        with open(fileName, 'rb') as f:
            data = pickle.load(f)
        # print(data[0])
        _scan = []
        for j in range(0, len(data)):
            # Wczytanie skanu z pierwszego zestawu danych
            _scan = []
            _scango = data[j]["scan"]
            _pose = data[j]["pose"]
            for k in range(0,len(_pose)):
                # print(type(_pose[k]))
                _pose[k]=round(_pose[k], 1)
            for i in range(0, len(_scango)):
                if (
                    not math.isnan(_scango[i])
                    and not  math.isinf(_scango[i])
                ):
                    _scan.append(_scango[i])
            # print(len(_scan))
            self.__rawdata.append((_pose, _scan))
            self.__map.append((_pose,np.histogram(_scan, range=(0.0, 10.0), bins=self.__binsNumber)[0]))
            self.__orientMap.append(self.generateOrientList(list(_scango)))

        self.makeHistMap()
        # print(len(self.__map))
        # print([self.__map[i][0] for i in range(0,len(self.__map))])
        # input()
    def makeHistMap(self):
        for i in self.__rawdata:
            # print(len(i[1]))
            hist, binno = np.histogram(
                i[1], range=(0.0, 10.0), bins=self.__binsNumber)
            self.__map.append((i[0], hist))
        

###################################################

    def generateOrientList(self, scan):
        orientList = []
        perSection = (len(scan))//self.__orientSectionNumber
        for i in range(0, self.__orientSectionNumber):
            sum = 0
            counter = 0
            for j in range(0, perSection):
                if (
                    not math.isnan(scan[i*perSection+j])
                    and not math.isinf(scan[i*perSection+j])
                ):
                    counter += 1
                    sum += scan[i*perSection+j]
                else:
                    counter+=1
            orientList.append((sum/counter))
                # print(sum/counter)
            # print(counter)
        return orientList
###################################################################################
############################ methods of neighbours comparation ####################

    
    def __histSubstract(self, h1,h2):
        diff = 0
        for j in range(0, self.__binsNumber):
            diff += abs(h1[j]-h2[j])
        return diff
    def __histMaxDifference(self, h1,h2):
        maxa,maxb = -1,-1
        maxdiff = 0 
        for j in range(0, self.__binsNumber):
            a = abs(h1[j]-h2[j])
            if (a>maxdiff):
                maxdiff = a
        return maxdiff
    
    def __histMinMaxDifference(self, h1,h2):
        maxdiff,mindiff = 0, 10000 
        for j in range(0, self.__binsNumber):
            tmp = abs(h1[j]-h2[j])
            if (tmp>maxdiff):
                maxdiff = tmp
            if tmp<mindiff:
                mindiff = tmp
        return (maxdiff+mindiff)/2.0
    
    def __histMeanDifference(self,h1, h2):
        diff = 0
        for j in range(0, self.__binsNumber):
            diff += abs(h1[j]-h2[j])
        return diff/self.__binsNumber

    def locateRobot(self, scanT: list=None,  G=(1.05, 1.1)):
        if scanT!=None and self.__ifSubscribing==False:
            tmpscan=scanT
        else:
            print(len(self.__scan))
            tmpscan = self.__scan
        hist1, b = np.histogram(tmpscan, range=(0.0, 10.0), bins=self.__binsNumber)
        diffList = []
        for i in self.__map:
            diff = 0
            for j in range(0, self.__binsNumber):
                diff += abs(i[1][j]-hist1[j])
            diffList.append(diff)
        diffList = np.array(diffList)



        ######################################################################
        ######################################################################
        ####### neighbours checking
        ######################################################################
        ######################################################################
        a = self.__map[np.argmin(diffList)][0]
        Apoint = self.__map[np.argmin(diffList)][1]

        searchListX = [[round(a[0]+self.__mapRes,1), round(a[1],1),0],[round(a[0]-self.__mapRes,1), round(a[1],1),0]]
        searchListY = [[round(a[0],1), round(a[1]+self.__mapRes,1),0],[round(a[0],1), round(a[1]-self.__mapRes,1),0]]

        pointsXList = []
        neighbXErrList = []
        pointsYList = []
        neighbYErrList = []
        xHist=[]
        yHist=[]
        for j in self.__map:
            for i in searchListX:
                if round(i[0],1)==round(j[0][0],1) and  round(i[1],1)==round(j[0][1],1) :
                    pointsXList.append(j[0])
                    neighbXErrList.append(self.__histSubstract(j[1],hist1))
                    xHist.append(j[1])
            for i in searchListY:
                if round(i[0],1)==round(j[0][0] ,1)and  round(i[1],1)==round(j[0][1],1 ):
                    pointsYList.append(j[0])
                    neighbYErrList.append(self.__histSubstract(j[1],hist1))
                    yHist.append(j[1])

        neighbXErrList = np.array(neighbXErrList)
        aX=np.argmin(neighbXErrList)
        neighbYErrList = np.array(neighbYErrList)
        aY=np.argmin(neighbYErrList)


        ######## orientation
        currentOrientList = self.generateOrientList(tmpscan)
        tmpOrient = self.__orientMap[np.argmin(diffList)].copy()
        copyCurrentList = currentOrientList.copy()

        sum = 0
        diffOrient = []
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

        orient = math.radians((len(diffOrient)-np.argmin(diffOrient))*(360//self.__orientSectionNumber))

        zX = self.__histSubstract(Apoint, xHist[aX])
        zY =  self.__histSubstract(Apoint,  yHist[aY])
        xiX = abs(1-(neighbXErrList[aX]/zX))*G[0]
        xiY = abs(1-(neighbYErrList[aY]/zY))*G[1]

        p = [round(self.__map[np.argmin(diffList)][0][0]+(pointsXList[aX][0] -self.__map[np.argmin(diffList)][0][0])*xiX,4),
             round(self.__map[np.argmin(diffList)][0][1]+(pointsYList[aY][1] - self.__map[np.argmin(diffList)][0][1])*xiY,4)]
        return LocalizationResult(p, orient, xiX, xiY)



    def locateRobotScan(self,scanT:list = None)->LocalizationResult:
        tmpscan = list(scanT).copy()
        hist1, b = np.histogram(tmpscan, range=(0.0, 10.0), bins=self.__binsNumber)
        diffList = []
        for i in self.__map:
            diff = 0
            diffList.append(self.__histSubstract(i[1],hist1))
            
        diffList = np.array(diffList)
        #####################################################
        # orientation
        currentOrientList = self.generateOrientList(tmpscan)
        tmpOrient = self.__orientMap[np.argmin(diffList)].copy()
        copyCurrentList = currentOrientList.copy()

        sum = 0
        diffOrient = []
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

        orient = math.radians((len(diffOrient)-np.argmin(diffOrient))*(360//self.__orientSectionNumber))

        return LocalizationResult(self.__map[np.argmin(diffList)][0],orient)
    
    
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
        self.__ifSubscribing=True
        rospy.init_node(self.__nodeName)
        rospy.Subscriber(topicName, LaserScan, self.callback)

    def exitTopicConnection(self):
        self.sub.unregister()
