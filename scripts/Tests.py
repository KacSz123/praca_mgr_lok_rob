from map_creation import mapCreation 
from hist_localization import HistogramLocalization
from probab_localization import ProbabLocalization
import pickle
from mymath import Dist2Points
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
# def twoPointsDist(a,b):
#    return math.sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))
MAP_DIR = './map/'
TEST_POINTS_DIR = './test-points/'
RESULTS_DIR='./results/'
def writeMapTofileJson(name):
    mfileName = name
    mapCreator = mapCreation(startPose=(12.4,-8.3), fileName=MAP_DIR+mfileName)
    mapCreator.initTopicConnection()
    mapCreator.writeMapToJsonFile(xRange=(14.8, 28.4), yRange=(-15.1, -13.6), resolution=(0.1,0.1))
def writeMapTofilePickle(name):
    mfileName = name
    mapCreator = mapCreation(startPose=(4, -12), fileName=MAP_DIR+mfileName)
    mapCreator.initTopicConnection()
    mapCreator.writeMapToPickleFile(xRange=(4, 11.2), yRange=(-12, -4.5), resolution=(0.1,0.1))

def writeTestPointsToPickle():
    mfileName = 'rand-1000pts-w-orient-p2'
    mapCreator = mapCreation(startPose=(4,-11.2,0), fileName=TEST_POINTS_DIR+mfileName)
    mapCreator.initTopicConnection()
    mapCreator.getRandomPointsToPickle((4, 11.2),(-12, -4.6), number=1000,precision=2)

def histLocalizationTest():
    errorList = []
    orientErrList = []
    hm=HistogramLocalization(bins=30,orientSection=30)
    with open('./test-points/rand-1000pts-w-orient-p2.pkl', 'rb') as f:
        data = pickle.load(f)
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    hm.loadMapPickle(fileName=mapFile)
    print(1)
    for i in data:
        print(i["pose"])
        p=hm.locateRobotScan(i["scan"])
        errorList.append(Dist2Points(p["point"],i["pose"]))
        orientErrList.append(abs(p["orientation"]-i["pose"][2]))

    #print(errorList)
    with open(RESULTS_DIR+'hist-Shelf-1000pts-local-p2-bins30.pkl', 'wb') as f:
        pickle.dump(errorList, f)
    with open(RESULTS_DIR+'hist-Shelf-1000pts-orient-p2-sec30.pkl', 'wb') as f:
        pickle.dump(orientErrList, f)
def generateStatisticsHist():
    with open('./results/hist-Shelf-1000pts-local-p2-bins30.pkl', 'rb') as f:
        data = pickle.load(f)
    s=pd.Series(data)
    print("localization")
    print(s.describe())
    with open('./results/hist-Shelf-1000pts-orient-p2-sec30.pkl', 'rb') as f:
        data2 = pickle.load(f)
    s2=pd.Series(data2)
    print("orientation")
    print(s2.describe())


def probabLocalizationOrientationTest():
    errorList = []
    orientErrList = []
    pm=ProbabLocalization(sectionsNumberLocalization=30, sectionsNumberOrient=30)
    with open('./test-points/rand-1000pts-w-orient-p2.pkl', 'rb') as f:
        data = pickle.load(f)
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    pm.loadMapLocalMinPickle(fileName=mapFile)
    print(1)
    for i in data:
        print(i["pose"])
        p,o=pm.locateRobotLocalMinimum(fileScan=i["scan"])
        errorList.append(Dist2Points(p,i["pose"]))
        orientErrList.append((abs(o-i["pose"][2])))
    #print(errorList)
    print(orientErrList)
    with open(RESULTS_DIR+'probab-Shelf-1000pts-local-p2-sec30.pkl', 'wb') as f:
        pickle.dump(errorList, f)
    with open(RESULTS_DIR+'probab-Shelf-1000pts-orient-p2-sec30.pkl', 'wb') as f:
        pickle.dump(orientErrList, f)

def generateStatisticsProbab():
    with open('./results/probab-Shelf-1000pts-local-p2-sec30.pkl', 'rb') as f:
        data = pickle.load(f)
    s=pd.Series(data)
    print("localization")
    print(s.describe())
    with open('./results/probab-Shelf-1000pts-orient-p2-sec30.pkl', 'rb') as f:
        data2 = pickle.load(f)
    s2=pd.Series(data2)
    print("orientation")
    print(s2.describe())

def showHistFromFile():
    with open('./results/probab-Shelf-1000pts-local-p2-sec25.pkl', 'rb') as f:
      data = pickle.load(f)
    x=np.array(data)
    plt.hist(x)
def __main__():
    probabLocalizationOrientationTest()
    # histLocalizationTest()
    # writeTestPointsToPickle()
    # histLocalizationTest()
    # generateStatisticsHist()
    generateStatisticsProbab()
#     fileName = './map/testMap.json'
#     hm = ProbabLocalization()
#     hm.initTopicConnection()
#     time.sleep(0.1)
#     hm.loadMapLocalMin(fileName)
#     print('###############################')
#     hm.locateRobotLocalMinimum()
#     print('###############################')

if __name__ == "__main__":
    __main__()