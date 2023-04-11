from map_creation import mapCreation 
from hist_localization import HistogramLocalization as hm
MAP_DIR = './map/'

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

def __main__():


    # writeMapTofileJson('mapRoomWithShelf-res01')
    writeMapTofilePickle('mapRoomWithShelf-res01')
    # hm.initTopicConnection()
    # time.sleep(0.1)
    
    
    # hm.loadMapLocalMin(fileName)
    # print('###############################')
    # hm.locateRobotLocalMinimum()
    # print('###############################')


if __name__ == "__main__":
    __main__()