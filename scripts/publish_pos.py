#!/usr/bin/python

import serial
import rospy
import glob
import threading
from geometry_msgs.msg import Pose
from time import sleep

stop_threading = False
position_data = []

class Pub:
    pos_publisher = rospy.Publisher("position", Pose, queue_size=10)
    pose = Pose()

    def __init__(self):
        rospy.init_node("Position_Publisher")

def getPort():
    ports = glob.glob('/dev/ttyACM*')
    if(len(ports) == 0):
        rospy.logfatal("No serial connection detected.")
        rospy.logfatal("Please check that the hardware is connected.")
        exit(1)
    elif(len(ports) > 1):
        print("Found ports:")
        for x in range(len(ports)):
            print(str(x) + " - " + ports[x])
        while True:
            try:
                port_num = int(raw_input("Select port number: "))
                if(port_num not in range(len(ports))):
                    continue
                break
            except ValueError:
                continue
        return ports[port_num]
    else:
        return ports[0]

def cmnd(cmnd):
    ser.write(cmnd)
    try:
        data = ser.readline()
    except:
        rospy.logwarn("Error Reading Serial Response: " + cmnd)
        return
    return data

def read_position_thread():
    while not stop_threading:
        try:
            serialData = cmnd("PX;")
            position_data.append(serialData)
        except:
            rospy.logwarn("Failed to Read Data")

def publish_position_thread():
    while not stop_threading:
        if len(position_data) <= 0:
            continue
        else:
            try:
                data = position_data.pop()
                position = float(data.split(";")[1])
                print("position: " + str(position))
                node.pose.position.x = position
                if node.pose.position.x:
                    node.pos_publisher.publish(node.pose)
            except:
                print("Failed to publish...")               

if __name__ == '__main__':
    node = Pub()
    baudrate = 115200
    time = 10  # seconds
    port = getPort()
    ser = serial.Serial(port, baudrate, timeout=1)
    ser.timeout = 0.0015  # based on unrestrained response time
    # '''
    t1 = threading.Thread(target=read_position_thread)
    t2 = threading.Thread(target=publish_position_thread)

    t1.start()
    t2.start()    

    sleep(time)

    stop_threading = True

    t1.join()
    t2.join()

    '''
    for x in range(10):
        try:
            # serialData = cmnd("PX;")
            ser.write("PX;")
            print(ser.read(5))
        except:
            rospy.logwarn("Failed to Read Data")
    print("Results:")
    for x in range(len(position_data)):
        print(str(x) + ": " + str(position_data[x]))
    '''


