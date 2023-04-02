from map_creation import mapCreation 
from hist_localization import HistogramLocalization as hm
MAP_DIR = './map/'
def __main__():
    mfileName = 'mapCorridorRightDown-res01.json'
    mapCreator = mapCreation(startPose=(12.4,-8.3), fileName=MAP_DIR+mfileName)
    mapCreator.initTopicConnection()
    mapCreator.writeMapToJsonFile(xRange=(14.8, 28.4), yRange=(-15.1, -13.6), resolution=(0.1,0.1))

    # hm.initTopicConnection()
    # time.sleep(0.1)
    
    
    # hm.loadMapLocalMin(fileName)
    # print('###############################')
    # hm.locateRobotLocalMinimum()
    # print('###############################')


if __name__ == "__main__":
    __main__()