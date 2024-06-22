#!/usr/bin/python
from cell_decomp import *
import rospy
import sys
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion
import math

def get_model_state(model_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        response = get_model_state_service(model_name, 'world')
        return response
    except rospy.ServiceException as e:
        print("Service call failed:", e)

def getPosition(data): 
    return[data.pose.position.x,data.pose.position.y]

def getOrientation(data):
    rot_q = data.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    return theta

def distance(x,y):
    return math.sqrt((x[0]-y[0])**2+(x[1]-y[1])**2)

def worldToMap(x):
    return [18.6*x[0]+360,-19.5*x[1]+280]

def mapToWorld(x):
    return [(x[1]-360)/18.6,(x[0]-280)/-19.5]

def angularVelocity(pos, theta, target,proportion=4):
    steer = math.atan2(target[1] - pos.y, target[0] - pos.x)
    return (proportion * (steer - theta))

def linearVelocity(pos,target,proportion=3):
    return proportion*math.sqrt((pos.x-target[0])**2+(pos.y-target[1])**2)

def moveToPost(post):
    while not rospy.is_shutdown():
        vel = Twist()
        vel.angular.x = 0
        vel.angular.y = 0
        vel.linear.x=0
        vel.linear.y=0    
        vel.angular.z = 0
        worldPos=mapToWorld(post)
        turtle_data = get_model_state("mobile_base")
        move_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)

        pos= getPosition(turtle_data)
        angle = getOrientation(turtle_data)

        if(distance(pos,worldPos<0.05)): #check if bot is close enough to post     
            move_pub.publish(vel)
            break

        #DO PID to move towards post
        #USE worldPos for goal in world coordinates

        steer = math.atan2(target[1] - pos.y, target[0] - pos.x)
        if(steer-angle>0.1):
            vel.angular.z = angularVelocity(pos,angle,worldPos)
            move_pub.publish(vel)
            continue
        
        vel.linear.x=linearVelocity(pos,worldPos)
        move_pub.publish(vel)

def Navigate(target):
    turtle_data = get_model_state("mobile_base")
    pos= getPosition(turtle_data)

    decomposedMap = segment_image_into_grid("map1.png", 7)
    path,newMap = AStar(worldToMap(pos).reverse(),worldToMap(target).reverse(),decomposedMap)

    print("Path found")

    for step in path:
        moveToPost(step)
    

#plt.title("Path")
#plt.imshow(newMap)

#plt.show()

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Parameter Usage: x y")
        sys.exit(1)

    target = [int(sys.argv[1]),int(sys.argv[2])]
    try:
        Navigate(target)
    except rospy.ROSInterruptException:
        pass