import numpy as np
import math
from scipy.stats import multivariate_normal

def twoPointsSubstraction(a,b):
    return abs((a[0]-b[0])+(a[1]-b[1]))

def get_yaw_z_degrees(x,y,z,w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(t3, t4))
def get_yaw_z_radians(x,y,z,w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)
def GetMuSigma(x):
    mu_x = np.mean(x)
    sigma_x = np.std(x)
    return mu_x, sigma_x*sigma_x
def gaus2d(x=0, y=0, mx=0, my=0, sx=1, sy=1):
    return 1. / (2. * np.pi * sx * sy) * np.exp(-((x - mx)**2. / (2. * sx**2.) + (y - my)**2. / (2. * sy**2.)))
def GetMuSigmaFromEqSqrt(x):
    suma=0
    for i in x:
        suma = suma + i
    mu_x=sum(x)/len(x)
    suma=0
    for i in x:
        suma = suma + (i-mu_x)*(i-mu_x)
    sigma_x=suma/len(x)
    return mu_x, math.sqrt(sigma_x)
        
def GetMuSigmaFromEq(x):
    suma=0
    for i in x:
        suma = suma + i
    mu_x=sum(x)/len(x)
    suma=0
    for i in x:
        suma = suma + (i-mu_x)*(i-mu_x)
    sigma_x=suma/len(x)
    return mu_x, sigma_x



    
def RegLinp(x,y,xmi,ymi):
    sumU=0
    sumD=0
    for i in range(len(x)):
        sumU+=(x[i]*y[i])
        sumD+=(x[i])**2
    sumD=(sumD-len(x)*(xmi**2))
    if sumD==0:
        sumD=0.001
    xr=(sumU-(len(x)*xmi*ymi))
    yr=sumD
    ang = 0
    a=xr/yr
    if xr<0 and yr>0:
        ang=math.degrees(math.atan2(xr,yr))
        if a<-1:
            ang=math.degrees(math.atan2(xr,yr)-math.pi/2)
    elif xr>0 and yr>0:            
        ang=math.degrees(math.atan2(xr,yr))
        if a>1:
            ang=math.degrees(math.atan2(xr,yr)+math.pi/2)
    else:            
        ang=math.degrees(math.atan2(xr,yr))              
    return ang

def SortBy2(el):
    return el[1]

def GetBeginEnd(x,y):
    ll=[]
    for i in range(len(x)):
        ll.append((x[i],y[i]))
    ll.sort(key=SortBy2)
    xr=x[0]-x[len(x)-1]
    yr=y[0]-y[len(y)-1]
    if xr==0:
        xr=0.01
    a=yr/xr

    if xr<0 and yr>0:
        ang=-math.degrees(math.atan2(xr,yr)+math.pi/2)
        if a<-1:
            ang=-math.degrees(math.atan2(xr,yr))
    elif xr>0 and yr>0:            
        ang=-math.degrees(math.atan2(xr,yr)-math.pi/2)
        if a>1:
            ang=-math.degrees(math.atan2(xr,yr))
                        #print('4hehe')
    else:            
        ang=-math.degrees(math.atan2(xr,yr))
    return ang

def Dist2Points(p1,p2):
    return math.sqrt(((p1[0]-p2[0])**2)+((p2[1]-p2[1])**2))