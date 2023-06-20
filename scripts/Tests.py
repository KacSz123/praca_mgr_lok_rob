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
    sec=200
    hm=HistogramLocalization(bins=sec,orientSection=25)
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
    Ge=(0.95,0.95)
    for i in range(0, len(data)):
        xlist.append(data[i]['pose'][0])
        ylist.append(data[i]['pose'][1])
        # xlist.append(data[i]['pose'][0])
        # print(i["pose"], 'rzeczywisty')
        p=hm.locateRobot(scanT=data[i]["scan"], G=Ge)
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
    with open(name[:-4]+'G'+str(Ge)+'-plotb'+str(sec)+'s.pkl', 'wb') as f:
        pickle.dump(okok, f)

def K_parameter_testsProbab(name):
    
    g_val=[(1.05,1.05)]
    sec=[100,200]

    for g in g_val:
        for s in sec:
            errorList = []
            orientErrList = []
            hm=ProbabLocalization(sectionsNumberLocalization=s,sectionsNumberOrient=30)
            with open(name, 'rb') as f:
                data = pickle.load(f)
            mapFile = './map/mapRoomWithShelf-res01.pkl'
            hm.loadMapLocalMinPickle(fileName=mapFile)
            print(1)
                # i = data[random.randint(0, 1000)]
            ksiX=[]
            ksiY=[]
            xlist=[]
            ylist=[]
            # xerr=[]
            # yerr=[]
            
            for i in range(0, len(data)):
                # time.sleep(0.5)
                xlist.append(data[i]['pose'][0])
                ylist.append(data[i]['pose'][1])
                # print(i["pose"], 'rzeczywisty')
                # print(i['pose'])
                p=hm.locateRobotLocalMinimum(fileScan=data[i]["scan"], G=g)
                errorList.append(Dist2Points(p["point"],data[i]["pose"]))
                orientErrList.append(abs(p["orientation"]-data[i]["pose"][2]))
                
                print(p['point'], 'skorygowany')
                print(data[i]['pose'], 'rzeczywisty')
                print()
                # xerr.append(abs(p['point'][1]-data[i]['pose'][1]))
                ksiX.append(p['ksiX'])
                ksiY.append(p['ksiY'])
            # plt.figure(1)75
            okok = {'ksiY':ksiY,'ksiX':ksiX, 'X':xlist, 'Y':ylist }
            with open(name[:-4]+'G-'+str(g)+'-plot-prob'+str(s)+'s.pkl', 'wb') as f:
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
    hm=HistogramLocalization(bins=200,orientSection=30)
    with open('./test-points/rand-1000pts-w-orient-p2.pkl', 'rb') as f:
        data = pickle.load(f)
    mapFile = './map/mapRoomWithShelf-res01.pkl'
    hm.loadMapPickle(fileName=mapFile)
    print(1)
        # i = data[random.randint(0, 1000)]
    for i in data:
        # print(i["pose"], 'rzeczywisty')
        p=hm.locateRobot(scanT=i["scan"],G=(0.00, 0.00))
        errorList.append(Dist2Points(p["point"],i["pose"]))
        orientErrList.append(abs(p["orientation"]-i["pose"][2]))
        # print(p['point'], 'obliczony')
        # pritn
        # print()
    # print(errorList)
    # print(orientErrList)
    with open("./someTests/hist-local-100-bins.pkl", 'wb') as f:
        pickle.dump(errorList, f)
    with open("./someTests/hist-local-100-bins.pkl", 'rb') as f:
        d = pickle.load(f)
    s=pd.Series(d)
    print("localization")
    print(s.describe())

def orientationCircle():
    histOrientFiles = ['results/hist-Shelf-1000pts-orient-p2-sec5.pkl','results/hist-Shelf-1000pts-orient-p2-sec10.pkl','results/hist-Shelf-1000pts-orient-p2-sec15.pkl',
                       'results/hist-Shelf-1000pts-orient-p2-sec20.pkl','results/hist-Shelf-1000pts-orient-p2-sec25.pkl','results/hist-Shelf-1000pts-orient-p2-sec30.pkl']
    histSaveFile = ['plot/histOrient/hist-orient-5s-p.png','plot/histOrient/hist-orient-10s-p.png','plot/histOrient/hist-orient-15s-p.png',
                    'plot/histOrient/hist-orient-20s-p.png','plot/histOrient/hist-orient-25s-p.png','plot/histOrient/hist-orient-30s-p.png']
    probabSaveFile = ['plot/histOrient/probab-orient-5s-p.png','plot/histOrient/probab-orient-10s-p.png','plot/histOrient/probab-orient-15s-p.png',
                    'plot/histOrient/probab-orient-20s-p.png','plot/histOrient/probab-orient-25s-p.png','plot/histOrient/probab-orient-30s-p.png']

    probabOrientfiles = ['results/probab-Shelf-1000pts-orient-p2-sec5.pkl','results/probab-Shelf-1000pts-orient-p2-sec10.pkl','results/probab-Shelf-1000pts-orient-p2-sec15.pkl',
                       'results/probab-Shelf-1000pts-orient-p2-sec20.pkl','results/probab-Shelf-1000pts-orient-p2-sec25.pkl','results/probab-Shelf-1000pts-orient-p2-sec30.pkl']

    for i in range(0,len(histOrientFiles)):
        with open(histOrientFiles[i], 'rb') as f:
            print(f.name)
            data = pickle.load(f)
        for j in range(0,len(data)):
            if data[j]>math.pi:
                data[j]=2*math.pi-data[j]

        plt.hist(np.array(data), bins=50, range=(0,math.pi))
        # plt.xlim((0,math.pi))
        plt.xlabel('Wartość błędu [rad]')
        plt.ylabel('Liczba wystąpień')
        # plt.show()
        plt.savefig(histSaveFile[i])
        plt.clf()
        s=pd.Series(data)
        print("localization")
        print(s.describe())
        # input()
        # data = load


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
    with open(RESULTS_DIR+'probab-local-neigh-5s.pkl', 'wb') as f:
        pickle.dump(errorList, f)
    with open(RESULTS_DIR+'probab-local-neigh-5s.pkl', 'wb') as f:
        pickle.dump(orientErrList, f)

def probabLocalizationOrientationTestNei():
    sections = [15,20,25,30,100]
    for sec in sections:
        errorList = []
        orientErrList = []
        pm=ProbabLocalization(sectionsNumberLocalization=sec, sectionsNumberOrient=30)
        with open('./test-points/rand-1000pts-w-orient-p2.pkl', 'rb') as f:
            data = pickle.load(f)
        mapFile = './map/mapRoomWithShelf-res01.pkl'
        pm.loadMapLocalMinPickle(fileName=mapFile)
        print(1)
        for i in data:
            # print(i["pose"]2
            p=pm.locateRobotLocalMinimum(fileScan=i["scan"], G=(0.95,0.95))
            errorList.append(Dist2Points(p['point'],i["pose"]))
            orientErrList.append((abs(p['orientation']-i["pose"][2])))
        #print(errorList)
        # print(orientErrList)
        with open(RESULTS_DIR+'probab-local-neigh-'+ str(sec)+'s.pkl', 'wb') as f:
            pickle.dump(errorList, f)
        # with open(RESULTS_DIR+'probab-local-neigh-5s.pkl', 'rb') as f:
        #     d = pickle.load(f)
        s=pd.Series(errorList)
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
    # probabLocalizationOrientationTestNei()
    # generateStatisticsHist()
    # generateStatisticsHist()
    # prezka()
    # histLocalizationTestImpr()
    # writeMapTofilePickleForKsi('point-XY_K_map')
    # K_parameter_tests('./map/point-XY_K_map.pkl')
    K_parameter_testsProbab('./map/point-XY_K_map.pkl')
    # orientationCircle()
if __name__ == "__main__":
    __main__()


    """
    count    1000.00
mean        0.162
std         0.3126
min         0.000
25%         0.040
50%         0.140
75%         0.250
max         7.100
    """