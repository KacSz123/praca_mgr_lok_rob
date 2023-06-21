


import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
from tf.transformations import quaternion_from_euler

import json
import pickle

import time
import numpy as np
import random
import math




class mapCreation():
    
    def __init__(self, modelName: str = 'myRobot', startPose: tuple = (-4, 1, 0), fileName: str = 'testMap'):
        rospy.wait_for_service('/gazebo/set_model_state')
        rospy.init_node('Mapping')

        self.__fileName = fileName
        self.__mapList = []
        self.__scan = []

        self.__set_state = rospy.ServiceProxy(
            '/gazebo/set_model_state', SetModelState)
        self.__state_msg = ModelState()

        self.__state_msg.model_name = modelName
        self.__state_msg.pose.position.x = startPose[0]
        self.__state_msg.pose.position.y = startPose[1]
        self.__state_msg.pose.position.z = 0.09999

        self.__setModelStationary()

        self.__resp = self.__set_state(self.__state_msg)
        
        self.__setRobotPosition(startPose)

    def __setModelStationary(self)->None:
        self.__state_msg.pose.orientation.x = 0
        self.__state_msg.pose.orientation.y = 0
        self.__state_msg.pose.orientation.z = 0
        self.__state_msg.pose.orientation.w = 0

        self.__state_msg.twist.linear.x = 0
        self.__state_msg.twist.linear.y = 0
        self.__state_msg.twist.linear.z = 0

        self.__state_msg.twist.angular.x = 0
        self.__state_msg.twist.angular.y = 0
        self.__state_msg.twist.angular.z = 0


    def __setRobotPosition(self, position: tuple) -> None:
        """ Set robot on position x=position[0], y=position[1] with orientation  θ=position[2]



        Args:
            position (tuple): (x,y,Theta), first two describes position 
            of robot on plane XY. Third one describes orientation of robot in radians.
        """
        quaternion = quaternion_from_euler(0, 0, position[2])
        self.__state_msg.pose.position.x = position[0]
        self.__state_msg.pose.position.y = position[1]

        self.__state_msg.pose.orientation.x = quaternion[0]
        self.__state_msg.pose.orientation.y = quaternion[1]
        self.__state_msg.pose.orientation.z = quaternion[2]
        self.__state_msg.pose.orientation.w = quaternion[3]

        self.__resp = self.__set_state(self.__state_msg)

    def writeMapToJsonFile(self, xRange, yRange, resolution=(0.5, 0.5)):
        """_summary_

        Args:
            xRange (_type_): _description_
            yRange (_type_): _description_
            resolution (tuple, optional): _description_. Defaults to (0.5, 0.5).
        """

        self.__mapList.clear()        
        for i in np.arange(xRange[0], xRange[1]+resolution[0]/5, resolution[0]):
            for j in np.arange(yRange[0], yRange[1]+resolution[1]/5, resolution[1]):
                self.__setRobotPosition((i, j, 0))
                time.sleep(0.3)
                self.__mapList.append({"pose": [i, j, 0], "scan": self.__scan})
        time.sleep(0.1)

        with open(self.__fileName+'.json', 'w') as f:
            json.dump(self.__mapList, f)

    def writeMapToPickleFile(self, xRange, yRange, resolution=(0.5, 0.5)) -> list:
        """_summary_

        Args:
            xRange (_type_): _description_
            yRange (_type_): _description_
            resolution (tuple, optional): _description_. Defaults to (0.5, 0.5).

        Returns:
            list: _description_
        """
        self.__mapList.clear()
        for i in np.arange(xRange[0], xRange[1]+resolution[0]/5, resolution[0]):
            for j in np.arange(yRange[0], yRange[1]+resolution[1]/5, resolution[1]):
                self.__setRobotPosition((i, j, 0))
                time.sleep(0.3)
                self.__mapList.append({"pose": [i, j, 0], "scan": self.__scan})

        time.sleep(0.1)
        name = self.__fileName+'.pkl'
        with open(name, 'wb') as f:
            pickle.dump(self.__mapList, f)
        return self.__mapList

    def getRandomPointsToPickle(self, xRange, yRange, number=1000, precision=3):
        pointsList = []
        while len(pointsList) <= number:
            randX = round(random.uniform(xRange[0], xRange[1]), precision)
            randY = round(random.uniform(yRange[0], yRange[1]), precision)
            angle = round(random.uniform(0, math.pi*2), 2)
            if (randX, randY) not in pointsList:
                pointsList.append((randX, randY, angle))

        self.__mapList.clear()
        for i in pointsList:
            print(i)
            self.__setRobotPosition(i)
            time.sleep(0.3)  # sleep to be sure of getting new data
            self.__mapList.append(
                {"pose": [i[0], i[1], i[2]], "scan": self.__scan})
        time.sleep(0.1)
        
        with open(self.__fileName+'.pkl', 'wb') as f:
            pickle.dump(self.__mapList, f)
            
        return self.__mapList

    def getRandomPointsToJson(): # to be added
        pass

    def initTopicConnection(self, topicName='/laser/scan'):
        rospy.Subscriber(topicName, LaserScan, self.callback)

    def callback(self, msg):
        self.__scan = msg.ranges
