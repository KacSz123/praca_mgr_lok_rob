import matplotlib.pyplot as plt
import pickle 
import numpy as np
import sys
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D


XPROB43_44 = ('point43-44X_K_mapG-0.7-plot-prob10s.pkl','point43-44X_K_mapG-0.7-plot-prob30s.pkl','point43-44X_K_mapG-0.7-plot-prob50s.pkl',
              'point43-44X_K_mapG-0.95-plot-prob10s.pkl','point43-44X_K_mapG-0.95-plot-prob30s.pkl','point43-44X_K_mapG-0.95-plot-prob50s.pkl',
              'point43-44X_K_mapG-1.05-plot-prob10s.pkl','point43-44X_K_mapG-1.05-plot-prob30s.pkl','point43-44X_K_mapG-1.05-plot-prob50s.pkl',
              'point43-44X_K_mapG-1.4-plot-prob10s.pkl','point43-44X_K_mapG-1.4-plot-prob30s.pkl','point43-44X_K_mapG-1.4-plot-prob50s.pkl',)
YPROB60_61 = ('point60-61Y_K_mapG-0.7-plot-prob10s.pkl','point60-61Y_K_mapG-0.7-plot-prob30s.pkl','point60-61Y_K_mapG-0.7-plot-prob50s.pkl',
              'point60-61Y_K_mapG-0.95-plot-prob10s.pkl','point60-61Y_K_mapG-0.95-plot-prob30s.pkl','point60-61Y_K_mapG-0.95-plot-prob50s.pkl',
              'point60-61Y_K_mapG-1.05-plot-prob10s.pkl','point60-61Y_K_mapG-1.05-plot-prob30s.pkl','point60-61Y_K_mapG-1.05-plot-prob50s.pkl',
              'point60-61Y_K_mapG-1.4-plot-prob10s.pkl','point60-61Y_K_mapG-1.4-plot-prob30s.pkl','point60-61Y_K_mapG-1.4-plot-prob50s.pkl',)

XPROB43_44P = ('point43-44X_K_mapG-0.7-plot-prob30s.pkl','point43-44X_K_mapG-0.95-plot-prob30s.pkl',
               'point43-44X_K_mapG-1.05-plot-prob30s.pkl','point43-44X_K_mapG-1.4-plot-prob30s.pkl')
YPROB60_61P = ('point60-61Y_K_mapG-0.7-plot-prob30s.pkl','point60-61Y_K_mapG-0.95-plot-prob30s.pkl',
               'point60-61Y_K_mapG-1.05-plot-prob30s.pkl','point60-61Y_K_mapG-1.4-plot-prob30s.pkl')
def plot2d():
        DIR = './map/'

        # name = DIR + 'point43-44X_K_mapG(0.7,0)-plotb.pkl'
        # savename =   'point43-44X_K_mapG(0.7,0)-plotb.png'
        for name in YPROB60_61:
                with open(DIR+name, 'rb') as f:
                        data = pickle.load(f)
                savename = name[:-4]+'.png'
                # print(len(data['ksi']))
                # print(len(data['zakres']))
                tmp = []
                # X = np.arange(0.0, 0.1, 0.001)

                # print(len(X))
                # print(len(tmp))
                for i in np.arange(0.0, 0.5, 0.01): 
                        tmp.append(round(i,2))
                if len(data['X'])==101:
                        for i in np.arange(0.501, 0.0, -0.01) : 
                                tmp.append(round(i,2))
                else:
                        for i in np.arange(0.50, 0.0, -0.01) : 
                                tmp.append(round(i,2))   
                
                plt.figure()
                
                plt.grid()
                plt.plot(data['Y'],tmp, color='r', label='Oczekiwany przebieg')
                plt.plot(data['Y'], data['ksiY'],'.',color='g', label='Rzeczywiste wartości ξy ')
                plt.ylabel('Wartość ξ')
                plt.xlabel('Pozycja na osi Y')
                plt.legend()
                plt.savefig('./plot/prob/Y/Y'+savename)
                plt.clf()

                # fig2=plt.figure(2)
                # plt.plot(data['Y'],tmp, color='b', label='Oczekiwany przebieg')
                # plt.plot(data['Y'], data['ksiY'],'.',color='r', label='Rzeczywiste wartości ξy')
                # plt.xlabel('Pozycja na osi Y')
                # plt.ylabel('Wartość ξ')
                # # plt.show(block=False)
                # plt.grid()
                # plt.savefig('Y'+savename)
                # plt.legend()
        # plt.show()


        # print(tmp)s

names = ('point-XY_K_mapG(0.95, 0.95)-plotb30s.pkl','point-XY_K_mapG(0.95, 0.95)-plotb50s.pkl',
         'point-XY_K_mapG(0.95, 0.95)-plotb100s.pkl','point-XY_K_mapG(0.95, 0.95)-plotb200s.pkl')
names2=('point-XY_K_mapG-(1.05, 1.05)-plot-prob30s.pkl','point-XY_K_mapG-(1.05, 1.05)-plot-prob50s.pkl',
        'point-XY_K_mapG-(1.05, 1.05)-plot-prob100s.pkl','point-XY_K_mapG-(1.05, 1.05)-plot-prob200s.pkl',)
def plot3d():
        DIR = './map/'

        # name = DIR + 'point-XY_K_mapG(1.2,1.2)-plot-prob.pkl'
        # savename = '3dpoint-XY_K_mapG(1.2,1.2)-plot-prob.png'
        for name in names2:
                with open(DIR+name, 'rb') as f:
                        data = pickle.load(f)
                savename='./plot/prob/'+name[-12:-4]+'.png'
                # print(len(data['ksi']))
                # print(len(data['zakres']))
                tmp = []
                # X = np.arange(0.0, 0.1, 0.001)

                # print(len(X))
                # print(len(tmp))
                for i in np.arange(0.0, 0.5, 0.01): 
                        tmp.append(round(i,2))
                if len(data['X'])==101:
                        for i in np.arange(0.501, 0.0, -0.01) : 
                                tmp.append(round(i,2))
                else:
                        for i in np.arange(0.50, 0.0, -0.01) : 
                                tmp.append(round(i,2))   
                ax = plt.figure().add_subplot(projection='3d')
                ax.view_init(elev=15, azim=-45)
                # Plot a sin curve using the x and y axes.
                print(len(data['X']))
                print(len(data['Y']))
                print(len(data['ksiX']))
                ax.scatter(data['X'], data['Y'], data['ksiX'], 'g',label='Rzeczywiste wartości ξx ')
                ax.scatter(data['X'], data['Y'], data['ksiY'], 'b',label='Rzeczywiste wartości ξy ')
                ax.plot(data['X'], data['Y'],tmp, 'r',label='Wartość oczekiwana ξ')
                # Plot scatterplot data (20 2D points per colour) on the x and z axes.
                # colors = ('r', 'g', 'b', 'k')

                # Fixing random state for reproducibility
                # np.random.seed(19680801)

                # x = np.random.sample(20 * len(colors))
                # y = np.random.sample(20 * len(colors))
                # c_list = []
                # for c in colors:
                #         c_list.extend([c] * 20)
                # # By using zdir='y', the y value of these points is fixed to the zs value 0
                # # and the (x, y) points are plotted on the x and z axes.
                # ax.scatter(x, y, zs=0, zdir='y', c=c_list, label='points in (x, z)')

                # # Make legend, set axes limits and labels
                # ax.legend()
                # ax.set_xlim(0, 1)
                # ax.set_ylim(0, 1)
                # ax.set_zlim(0, 1)
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('ξ')
                ax.legend()
                # Customize the view angle so it's easier to see that the scatter points lie
                # # on the plane y=0
                # ax.autoscale_view()
                
                plt.savefig(savename)
        # plt.sho1(2
plot3d()
# plot2d()