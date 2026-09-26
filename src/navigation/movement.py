from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
import time


node = Node()

publisher = node.advertise(
    "/model/curious_robot/cmd_vel",
    Twist
)

time.sleep(1)


def move_forward():
    msg = Twist()
    msg.linear.x = 0.5
    msg.angular.z = 0.0
    publisher.publish(msg)

def stop():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    publisher.publish(msg)

def turn_left():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.5
    publisher.publish(msg)

def turn_right():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = -0.5
    publisher.publish(msg)

def turn_around():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 1.0
    publisher.publish(msg)

def move_backward():
    msg = Twist()
    msg.linear.x = -0.5
    msg.angular.z = 0.0
    publisher.publish(msg)

def move(linear_speed, angular_speed):
    msg = Twist()
    msg.linear.x = linear_speed
    msg.angular.z = angular_speed
    publisher.publish(msg)


# z = input("Enter a command (w: forward, s: backward, a: left, d: right, x: stop, t: turn around, q: quit): ")
# while z != "q":
#     if z == "w":
#         move_forward()
#     elif z == "s":
#         move_backward()
#     elif z == "a":
#         turn_left()
#     elif z == "d":
#         turn_right()
#     elif z == "x":
#         stop()
#     elif z == "t":
#         turn_around()
#     else:
#         print("Invalid command. Please try again.")
    
#     z = input("Enter a command (w: forward, s: backward, a: left, d: right, x: stop, t: turn around, q: quit): ")