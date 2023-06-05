import rospy 
import json
import pickle
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import geometry_msgs
import time
import numpy as np
import random
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# from mymath import *
class mapCreation():
    def __init__(self, modelName='myRobot', startPose=(-4,1,0),fileName = 'testMap'):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.init_node('Mapping')
        self.__fileName=fileName
        self.__mapList = []
        self.__set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.__state_msg = ModelState()
        self.__state_msg.model_name = modelName
        self.__state_msg.pose.position.x = startPose[0]
        self.__state_msg.pose.position.y = startPose[1]
        self.__state_msg.pose.position.z = 0.09999
        self.__state_msg.pose.orientation.x = 0
        self.__state_msg.pose.orientation.y = 0
        self.__state_msg.pose.orientation.z = 0
        self.__state_msg.pose.orientation.w = 0
        self.__state_msg.twist.linear.x=0
        self.__state_msg.twist.linear.y=0
        self.__state_msg.twist.linear.z=0
        self.__state_msg.twist.angular.x=0
        self.__state_msg.twist.angular.y=0
        self.__state_msg.twist.angular.z=0
        self.__resp = self.__set_state( self.__state_msg )
        self.__setRobotPosition(startPose)
        self.__scan = []
        print(self.__fileName)
    
    def __setRobotPosition(self, position):
        quaternion = quaternion_from_euler(0,0,position[2])
        # print(quaternion)
        self.__state_msg.pose.position.x = position[0]
        self.__state_msg.pose.position.y = position[1]
        self.__state_msg.pose.orientation.x=quaternion[0]
        self.__state_msg.pose.orientation.y=quaternion[1]
        self.__state_msg.pose.orientation.z=quaternion[2]
        self.__state_msg.pose.orientation.w=quaternion[3]
        self.__resp = self.__set_state( self.__state_msg )

    def writeMapToJsonFile(self, xRange, yRange, resolution=(0.5,0.5)):
        print(1)
        self.__mapList.clear()
        for i in np.arange(xRange[0],xRange[1],resolution[0]):
            for j in np.arange(yRange[0],yRange[1],resolution[1]):
                self.__setRobotPosition((i,j,0))
                time.sleep(0.3)
                self.__mapList.append({"pose":[i,j,0],"scan":self.__scan})
        time.sleep(0.5)
        #jsonStr = json.dumps(self.__mapList)
        with open(self.__fileName+'.json', 'w') as f:
             json.dump(self.__mapList, f)
        # print(1)

    def writeMapToPickleFile(self, xRange, yRange, resolution=(0.5,0.5), fileName=None):
        print(1)
        self.__mapList.clear()
        for i in np.arange(xRange[0],xRange[1],resolution[0]):
            for j in np.arange(yRange[0],yRange[1]+resolution[1]-resolution[1]/10,resolution[1]):
                self.__setRobotPosition((round(i,4),round(j,4),0.000))
                # print(j)
                time.sleep(0.4)
                self.__mapList.append({"pose":[round(i,4),round(j,4),0],"scan":self.__scan})
        time.sleep(0.7)
        if fileName==None:
            name=self.__fileName+'.pkl'
        else:
            name==fileName+'.pkl'
        with open(name, 'wb') as f:
             pickle.dump(self.__mapList, f)
        
        return self.__mapList


    def getRandomPointsToPickle(self, xRange, yRange, number=1000, precision=1):
        pointsList=[]
        while len(pointsList)<number:
            randX=round(random.uniform(xRange[0], xRange[1]), precision)
            randY=round(random.uniform(yRange[0], yRange[1]), precision)
            angle=round(random.uniform(0, math.pi*2),2)
            if (randX,randY) not in pointsList:
                pointsList.append((randX,randY,angle))
        print(1)
        self.__mapList.clear()
        for i in pointsList:
                print(i)
                self.__setRobotPosition(i)
                time.sleep(0.3)
                self.__mapList.append({"pose":[i[0],i[1],i[2]],"scan":self.__scan})
        time.sleep(0.5)
        #jsonStr = json.dumps(self.__mapList)
        with open(self.__fileName+'.pkl', 'wb') as f:
             pickle.dump(self.__mapList, f)
        # print(len(pointsList))
        # print(pointsList)
        return self.__mapList
    def getRandomPointsToJson():
        return

    def initTopicConnection(self, topicName='/laser/scan'):
        rospy.Subscriber(topicName, LaserScan, self.callback)
        
    def callback(self,msg):
        self.__scan = msg.ranges

# def __main__():
#     myMap=mapCreation()
#     myMap.initTopicConnection()
#     myMap.writeMapToJsonFile((-4,-1),(3, 4.5))
#     # rospy.spin()


# if __name__ == '__main__':
#     try:
#         __main__()
#     except rospy.ROSInterruptException:
#         pass