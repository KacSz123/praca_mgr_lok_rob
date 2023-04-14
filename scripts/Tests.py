from map_creation import mapCreation 
from hist_localization import HistogramLocalization
import pickle
import mymath
import math
def twoPointsDist(a,b):
    return math.sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))
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
    mfileName = 'random1000points-corridor'
    mapCreator = mapCreation(startPose=(12.4,-8.3), fileName=TEST_POINTS_DIR+mfileName)
    mapCreator.initTopicConnection()
    mapCreator.getRandomPointsToPickle(xRange=(14.8, 28.4),yRange=(-15.1, -13.6))

def histLocalizationTest():
    errorList = []
    hm=HistogramLocalization()
    with open('./test-points/random1000points.pkl', 'rb') as f:
        data = pickle.load(f)
    # print(data[0]["scan"])
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    hm.loadMapPickle(fileName=mapFile)
    print(1)
    for i in data:
        p=hm.locateRobotScan(i["scan"])
        # print(i["pose"])
        # print(p["point"])
        errorList.append(twoPointsDist(p["point"],i["pose"]))
    print(errorList)
    with open(RESULTS_DIR+'histLocalShelf1000.pkl', 'wb') as f:
        pickle.dump(errorList, f)
def histOrientationTest():
    return
def probabLocalizationTest():
    return
def probabOrientationTest():
    return

def __main__():
    histLocalizationTest()
    #writeTestPointsToPickle()
    # writeMapTofileJson('mapRoomWithShelf-res01')
    #writeMapTofilePickle('mapRoomWithShelf-res01')
    # hm.initTopicConnection()
    # time.sleep(0.1)
    
    
    # hm.loadMapLocalMin(fileName)
    # print('###############################')
    # hm.locateRobotLocalMinimum()
    # print('###############################')


if __name__ == "__main__":
    __main__()