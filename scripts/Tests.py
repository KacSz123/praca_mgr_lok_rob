from map_creation import mapCreation 
from hist_localization import HistogramLocalization
from probab_localization import ProbabLocalization
import pickle
import signal
from mymath import Dist2Points
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import random
import sys
import time
# def twoPointsDist(a,b):
#    return math.sqrt(pow(a[0]-b[0],2)+pow(a[1]-b[1],2))
MAP_DIR = './map/'
TEST_POINTS_DIR = './test-points/'
RESULTS_DIR='./results/'


def handler(signum, frame):
    res = input("\n\nCtrl-c was pressed. Do you really want to exit? y/n ")
    if res == 'y':
        exit(1)
signal.signal(signal.SIGINT, handler)

def K_parameter_tests(name):
    errorList = []
    orientErrList = []
    hm=HistogramLocalization(bins=30,orientSection=25)
    with open(name, 'rb') as f:
        data = pickle.load(f)
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    hm.loadMapPickle(fileName=mapFile)
    print(1)
        # i = data[random.randint(0, 1000)]
    ksiX=[]
    ksiY=[]
    xlist=[]
    ylist=[]
    xerr=[]
    yerr=[]
    for i in range(0, len(data)):
        xlist.append(data[i]['pose'][0])
        ylist.append(data[i]['pose'][1])
        # xlist.append(data[i]['pose'][0])
        # print(i["pose"], 'rzeczywisty')
        p=hm.locateRobot(scanT=data[i]["scan"], G=(1.20, 1.20))
        errorList.append(Dist2Points(p["point"],data[i]["pose"]))
        orientErrList.append(abs(p["orientation"]-data[i]["pose"][2]))
        
        print(p['point'], 'skorygowany')
        print(data[i]['pose'], 'rzeczywisty')
        print()
        xerr.append(abs(p['point'][1]-data[i]['pose'][1]))
        ksiX.append(p['ksiX'])
        ksiY.append(p['ksiY'])
    plt.figure(1)
    plt.plot(xlist, ksiX,'.',color='r')
    plt.plot(xlist, ksiY,'+',color='g')
    tmp=[]
    for i in np.arange(0.0, 0.5, 0.01): 
        tmp.append(round(i,2))
    for i in np.arange(0.501, 0.0, -0.01) : 
        tmp.append(round(i,2))
    plt.plot(xlist, tmp,color='b')
    plt.grid()
    # plt.title('Wartość parametru ξ dla:'+str(data[0]['pose'][0])+' do '+str(data[len(data)-1]['pose'][0]))
    plt.xlabel('Pozycja robota na osi Y')
    plt.ylabel('Wartość parametru ξ')
    plt.show()
    okok = {'ksiX':ksiX, 'ksiY':ksiY, 'Y':ylist, 'X':xlist}
    with open(name[:-4]+'G(1.20,1.20)-plotb.pkl', 'wb') as f:
        pickle.dump(okok, f)

def K_parameter_testsProbab(name):
    
    errorList = []
    orientErrList = []
    hm=ProbabLocalization(sectionsNumberLocalization=30,sectionsNumberOrient=30)
    with open(name, 'rb') as f:
        data = pickle.load(f)
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    hm.loadMapLocalMinPickle(fileName=mapFile)
    print(1)
        # i = data[random.randint(0, 1000)]
    ksiX=[]
    ksiY=[]
    xlist=[]
    xerr=[]
    yerr=[]
    
    for i in range(0, len(data)):
        # time.sleep(0.5)
        xlist.append(data[i]['pose'][0])
        # print(i["pose"], 'rzeczywisty')
        p=hm.locateRobotLocalMinimum(fileScan=data[i]["scan"], G=(0.5, 0.5))
        errorList.append(Dist2Points(p["point"],data[i]["pose"]))
        orientErrList.append(abs(p["orientation"]-data[i]["pose"][2]))
        
        print(p['point'], 'skorygowany')
        print(data[i]['pose'], 'rzeczywisty')
        print()
        xerr.append(abs(p['point'][1]-data[i]['pose'][1]))
        ksiX.append(p['ksiX'])
        ksiY.append(p['ksiY'])
    plt.figure(1)
    plt.plot(xlist, ksiX,'.',color='r')
    plt.plot(xlist, ksiY,'+',color='g')
    tmp=[]
    for i in np.arange(0.0, 0.5, 0.01): 
        tmp.append(round(i,2))
    for i in np.arange(0.50, 0.0, -0.01) : 
        tmp.append(round(i,2))
    plt.plot(xlist, tmp,color='b')
    plt.grid()
    # plt.title('Wartość parametru ξ dla:'+str(data[0]['pose'][0])+' do '+str(data[len(data)-1]['pose'][0]))
    plt.xlabel('Pozycja robota na osi Y')
    plt.ylabel('Wartość parametru ξ')
    plt.show()
    okok = {'ksi':ksiY, 'zakres':xlist}
    with open(name[:-4]+'0.98G-plot-prob.pkl', 'wb') as f:
        pickle.dump(okok, f)

    # plt.figure(2)
    # plt.plot(xlist, xerr)
    # plt.show()
    # input()
    # ksi=[]
    # xlist=[]
    # xerr=[]
    # # for i in range(len(data)//2, len(data)):
    # #     xlist.append(data[i]['pose'][0])
    # #     # print(i["pose"], 'rzeczywisty')
    # #     p=hm.locateRobot(scanT=data[i]["scan"])
    # #     errorList.append(Dist2Points(p["point"],data[i]["pose"]))
    # #     orientErrList.append(abs(p["orientation"]-data[i]["pose"][2]))
        
    # #     print(p['point'], 'skorygowany')
    # #     print(data[i]['pose'], 'rzeczywisty')
    # #     print()
    # #     xerr.append(abs(p['point'][0]-data[i]['pose'][0]))
    # #     ksi.append(p['ksiX'])
    # # plt.figure(1)
    # # plt.plot(xlist, ksi,'.')
    # # plt.xlabel('Pozycja robota na osi X')
    # # plt.xlabel('Wartość parametru ξ')

    # # plt.title('Wartość parametru ξ dla: '+str(data[len(data)//2+1]['pose'][0])+'do'+str(data[-1]['pose'][0]))
    # # plt.show()
    # # plt.figure(2)
    # # plt.plot(xlist, xerr,'.')
    # # plt.show()
    # input()
        # print(p['point'], 'obliczony')
        # pritn
        # print()
    # print(errorList)
    # print(orientErrList)
    # with open("./someTests/hist-debug2.pkl", 'wb') as f:
    #     pickle.dump(errorList, f)
    # with open("./someTests/hist-debug2.pkl", 'rb') as f:
    #     d = pickle.load(f)
    # s=pd.Series(d)
    # for i in data:
    #     print(i["pose"])
    # print(len(data))
    # print
        # i = data[random.randint(0, 1000)]

    # i  =  random.choice(data)
    
    # print(i["pose"], 'rzeczywisty')
    # p=hm.locateRobot(scanT=i["scan"])
    # # Kerror.append(Dist2Points(p["point"],i["pose"]))
    # # orientErrList.append(abs(p["orientation"]-i["pose"][2]))
    # # print([round(i,2) for i in p['point']], 'obliczony')
    
    # print(i["pose"])
    # print(p["p_origin"], 'origin')
    # pritn
    # print()
    # print(Kerror)
    # print(orientErrList)
    # with open("./someTests/Kx-debug.pkl", 'wb') as f:
    #     pickle.dump(Kerror, f)
    # with open("./someTests/hist-debug2.pkl", 'rb') as f:
    #     d = pickle.load(f)
    # s=pd.Series(d)
    # print("localization")
    # print(s.describe())
    


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

def writeMapTofilePickleForKsi(name):
    mfileName = name
    mapCreator = mapCreation(startPose=(5.24000, -8.05000,0.000), fileName=MAP_DIR+mfileName)
    mapCreator.initTopicConnection()
    mapCreator.writeMapToPickleFile(xRange=(6.3000,6.400000), yRange=(-10.10000, -10.0000), resolution=(0.001,0.001))

def writeTestPointsToPickle():
    mfileName = 'rand-1000pts-w-orient-p2'
    mapCreator = mapCreation(startPose=(4,-11.2,0), fileName=TEST_POINTS_DIR+mfileName)
    mapCreator.initTopicConnection()
    mapCreator.getRandomPointsToPickle((4, -11.2),(-12, -4.5), number=1000,precision=2)
    
def prezka():
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    with open('./map/mapRoomWithShelf-res01.pkl', 'rb') as f:
        data = pickle.load(f)
    for i in data:
        # print(type(i["pose"]))
        # input()
        if [round(j,1) for j in i["pose"]]==[10.0,-10.0,0]:
            plt.hist(i["scan"], range = (0,10), bins=25)
            plt.xlabel('Odległośc odczytana z lasera [m]')
            plt.ylabel('Liczba wystąpień')
            plt.show()
            input()
            sys.exit()
    pass
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
        # print(i["pose"])
        p=hm.locateRobotScan(i["scan"])
        errorList.append(Dist2Points(p["point"],i["pose"]))
        orientErrList.append(abs(p["orientation"]-i["pose"][2]))
    #print(errorList)
    # with open(RESULTS_DIR+'hist-debug-local.pkl', 'wb') as f:
    #     pickle.dump(errorList, f)
    with open(RESULTS_DIR+'hist-Shelf-1000pts-orient-p2-sec30.pkl', 'wb') as f:
        pickle.dump(orientErrList, f)


def histLocalizationTestImpr():
    errorList = []
    orientErrList = []
    hm=HistogramLocalization(bins=50,orientSection=30)
    with open('./test-points/rand-1000pts-w-orient-p2.pkl', 'rb') as f:
        data = pickle.load(f)
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    hm.loadMapPickle(fileName=mapFile)
    print(1)
        # i = data[random.randint(0, 1000)]
    for i in data:
        # print(i["pose"], 'rzeczywisty')
        p=hm.locateRobot(scanT=i["scan"],G=(0.80, 0.70))
        errorList.append(Dist2Points(p["point"],i["pose"]))
        orientErrList.append(abs(p["orientation"]-i["pose"][2]))
        # print(p['point'], 'obliczony')
        # pritn
        # print()
    # print(errorList)
    # print(orientErrList)
    with open("./someTests/hist-debug2.pkl", 'wb') as f:
        pickle.dump(errorList, f)
    with open("./someTests/hist-debug2.pkl", 'rb') as f:
        d = pickle.load(f)
    s=pd.Series(d)
    print("localization")
    print(s.describe())


def generateStatisticsHist():
    # with open('./results/hist-debug-local.pkl', 'rb') as f:
    #     data = pickle.load(f)
    # s=pd.Series(data)
    # print("localization")
    # print(s.describe())
    with open('./results/hist-Shelf-1000pts-orient-p2-sec25.pkl', 'rb') as f:
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
    with open(RESULTS_DIR+'probab-Shelf-1000pts-local-p2-sec30b.pkl', 'wb') as f:
        pickle.dump(errorList, f)
    with open(RESULTS_DIR+'probab-Shelf-1000pts-orient-p2-sec30b.pkl', 'wb') as f:
        pickle.dump(orientErrList, f)

def probabLocalizationOrientationTestNei():
    errorList = []
    orientErrList = []
    pm=ProbabLocalization(sectionsNumberLocalization=5, sectionsNumberOrient=30)
    with open('./test-points/rand-1000pts-w-orient-p2.pkl', 'rb') as f:
        data = pickle.load(f)
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    pm.loadMapLocalMinPickle(fileName=mapFile)
    print(1)
    for i in data:
        # print(i["pose"]2
        p,o=pm.locateRobotLocalMinimum(fileScan=i["scan"])
        errorList.append(Dist2Points(p,i["pose"]))
        orientErrList.append((abs(o-i["pose"][2])))
    #print(errorList)
    print(orientErrList)
    with open("./someTests/ProbabImprNei5-o3.pkl", 'wb') as f:
        pickle.dump(errorList, f)
    with open('./someTests/ProbabImprNei5-o3.pkl', 'rb') as f:
        d = pickle.load(f)
    s=pd.Series(d)
    print("localization")
    print(s.describe())

def generateStatisticsProbab():
    with open('./results/probab-Shelf-1000pts-local-p2-sec30b.pkl', 'rb') as f:
        data = pickle.load(f)
    s=pd.Series(data)
    print("localization")
    print(s.describe())
    with open('./results/probab-Shelf-1000pts-orient-p2-sec30b.pkl', 'rb') as f:
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
    # probabLocalizationOrientationTest()
    # generateStatisticsProbab()
    # histLocalizationTest()
    #probabLocalizationOrientationTestNei()
    # generateStatisticsHist()
    # generateStatisticsHist()
    # prezka()
    # histLocalizationTestImpr()
    # writeMapTofilePickleForKsi('point-XY_K_map')
    K_parameter_tests('./map/point-XY_K_map.pkl')
    # K_parameter_testsProbab('./map/point49-50X_K_map.pkl')
if __name__ == "__main__":
    __main__()