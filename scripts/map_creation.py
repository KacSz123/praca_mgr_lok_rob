import rospy 
import json
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
import time
import numpy as np
class mapCreation():
    def __init__(self, modelName='myRobot', startPose=(-4,1),fileName = 'testMap.json'):
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
        self.__state_msg.pose.position.x = position[0]
        self.__state_msg.pose.position.y = position[1]
        self.__resp = self.__set_state( self.__state_msg )

    def writeMapToJsonFile(self, xRange, yRange, resolution=(0.5,0.5)):
        print(1)
        self.__mapList.clear()
        for i in np.arange(xRange[0],xRange[1],resolution[0]):
            for j in np.arange(yRange[0],yRange[1],resolution[1]):
                self.__setRobotPosition((i,j))
                time.sleep(0.3)
                self.__mapList.append({"pose":[i,j,0],"scan":self.__scan})
        time.sleep(0.5)
        jsonStr = json.dumps(self.__mapList)
        with open(self.__fileName, 'w') as f:
             json.dump(self.__mapList, f)
        print(1)

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