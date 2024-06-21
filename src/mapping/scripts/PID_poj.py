##!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

class PID():

    def __init__(self,kp,ki,kd,model_name):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.model_name=model_name

    def get_model_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            response = get_model_state_service(self.model_name, 'world')
            return response
        except rospy.ServiceException as e:
            print("Service call failed:", e)

    def get_rotation(self, state):
        orientation_q = state.pose.orientation  
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]  
        (roll, pitch, yaw) = euler_from_quaternion(
            orientation_list)
        return yaw

    def linear_pid(self, error):
        self.error = error  
        self.integral_error += self.error  
        self.derivative_error = self.error - self.error_last  
        self.error_last = self.error  
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error  
        return self.output

    def angular_pid(self, yaw):
        self.error = 0-yaw  
        self.integral_error += self.error  
        self.derivative_error = self.error - self.error_last
        self.error_last = self.error  
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error  
        return self.output

    def getPosition(data): 
        return[data.pose.position.x,data.pose.position.y,data.pose.position.z]

    def getOrientation(data):
        return[data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z]

    def getPosError(position,target):
        return[target[0]-position[0],target[1]-position[1],target[2]-position[2]]

    def getOError(orientation):
        return[-1*orientation[0],-1*orientation[1],-1*orientation[2]]
    
