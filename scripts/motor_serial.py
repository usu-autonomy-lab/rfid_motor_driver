#!/usr/bin/python

import serial
import rospy
import glob
from geometry_msgs.msg import Pose
import gui
import subprocess
from numpy import pi
from time import sleep, localtime, strftime
import threading

baudrates = [4800, 9600, 19200, 38400, 57600, 115200, 230400]
baudrate = baudrates[6]  # use baudrates[1] (9600 baud) 


class Pub:
    pos_publisher = rospy.Publisher("position", Pose, queue_size = 10)
    pose = Pose()

    def __init__(self):
        rospy.init_node("Position_Publisher")

def check_ports():
    """     function that checks available serial ports 
            and returns the selected port                  """

    ports = glob.glob('/dev/ttyACM*')

    if(len(ports) == 0):
        rospy.logfatal("No serial connections detected.")
        rospy.logfatal("Please check that the hardware is connected correctly.")
        exit(1)
    elif(len(ports) > 1):
        print("Found Ports:")
        for x in range(len(ports)):
            print(str(x) + " - " + ports[x])
        while True:
            try:
                port_num = int(raw_input("Please select a valid port number: "))
                # port_num = 1  # just to set default for testing
                if(port_num not in range(len(ports))):
                    continue
                break
            except ValueError:
                continue
        return ports[port_num]
    else:
        return ports[0]

def homing(ser, current_limit):
    """     function that runs initially and sets the homing position   """
    pass
    
def broadcast():
    """     function that request current position from serial device, parses response, and 
            broadcasts to ROS topic in motor counts """
    while not rospy.is_shutdown():
        try:
            serialData = cmnd("PX;")
            data = serialData.split(";")
            position = float(data[1])
        except:
            rospy.logwarn("Failed to Read and Parse Position")
        
        try:
            node.pose.position.x = position
            if node.pose.position.x:
                node.pos_publisher.publish(node.pose)
                print("published: " + str(node.pose.position.x))
        except serial.serialutil.SerialException: 
            ser.close()
            break
        except ValueError:
            continue

        rate.sleep()

def countdown(motion_time):
    while motion_time > 0:
        sleep(1)
        motion_time -= 1

def cmnd(cmnd):
    '''send cmnd over serial connection, returns response from device'''
    ser.write(cmnd)
    try:
        data = ser.readline()
    except:
        rospy.logwarn("Error Reading Serial Response After Command: " + cmnd)
        return
    return data

if __name__ == '__main__':
    params = gui.gui()  # [iter,accel,decel,speed,current_limit,
                        #  bench_x,bench_y,bench_z,bench_angle, publish rate]
    """ errors within the following roslaunch file, removed for test...
    subprocess.call(['roslaunch', 'bench_tf2', 'bench_test.launch', 
                    'x:='+str(params[5]), 'y:='+str(params[6]),
                    'z:='+str(params[7]), 'angle_rad:='+str(params[8] * 180 / pi)])
    """
    node = Pub()
    rate = rospy.Rate(params[9])

    port = check_ports()
    ser = serial.Serial(port, baudrate, timeout = 1)

    # print("Included Homing...")
    # homing(ser, params[4])  # begin homing procedure

    maxVelocity = "1000" # default: 1000, steps/second
    maxAcceleration = "600000"  # default: 600000, 600000 max
    motion_time = 10 # duration of motion (seconds)
    toPrintDetails = True
    STEPS_IN_METER = 75187.9699
    maxStepsPerSec = int(maxVelocity) * STEPS_IN_METER

    if int(maxAcceleration) > 600000:
        rospy.logwarn("Max Acceleration Exceeds 600,000. Risks Overvoltage to Controller. Do not Proceed.")

    if toPrintDetails:
        print("Port: " + ser.portstr)
        print("Baudrate: " + str(baudrate))

    gotInput = False
    while not gotInput:
        test_selection = raw_input("Direct Commands (1) or Repetative Motion (2): ")
        if test_selection is "1" or "0":
            gotInput=True

    if test_selection is "1":
        print("Entered Direct Command, Enter 'exit()' to exit")
        while test_selection is "1":
            userCommand = raw_input(">> ")
            if userCommand == "exit()":
                test_selection = "x"
            else:
                print("<< " + cmnd(userCommand))
    else:
        MR_mode = "2"  # default: 2, point to point repetative motion
        MR_delay = "0"  # default: 0, delay after finished cycle 
        MR_first = "10"  # default: 0, beginning position
        MR_second = "-10"  # default: 1000, second position
        try:
            print("Configuring Motion Controller...")
            cmnd("MO=0;")
            resultMaxAccel = cmnd("AC="+maxAcceleration+";")
            resultMaxDecel = cmnd("DC="+maxAcceleration+";")
            cmnd("MO=1;")
            resultMode = cmnd("MR[1]="+MR_mode+";")
            resultDelay = cmnd("MR[2]="+MR_delay+";")
            resultFirst = cmnd("MR[3]="+MR_first+";")
            resultSecond = cmnd("MR[4]="+MR_second+";")
            resultMaxVel = cmnd("VH[2]="+maxVelocity+";")
            cmnd("MO=0;")
        except:
            rospy.logfatal("failed to set rep. motion, exiting...")
            exit()
        if(toPrintDetails):
            # print("Repetition Details --------------------")
            print("Mode :     "+resultMode)
            print("Delay:     "+resultDelay)
            print("Posit A:   "+resultFirst)
            print("Posit B:   "+resultSecond)
            print("Max Vel:   "+resultMaxVel)
            print("Max Accel: "+resultMaxAccel)
            print("Max Decel: "+resultMaxDecel)
            print("Duration:  "+str(motion_time)+" seconds")
            # print("----------------------------------------\n")

        if raw_input("System Configured, Begin? (yes/no): ") != "yes":
            exit()
        print("Send Commands to Motor...")

        data = cmnd("MO=1;")
        if data == "MO=0;;":
            rospy.logfatal("  Motor failed to initialize:  " + data)
            exit()
        print("  Beginning Motion...")
        cmnd("MO=1;")   
        cmnd("BG;")

    # t1 = threading.Thread(target=broadcast)
    # t2 = threading.Thread(target=countdown, args=[motion_time])
    # t1.start()
    # t2.start()

    # while not rospy.is_shutdown():
    #     t2.join(timeout=0.1)

    if motion_time <= 0:
        print("  End of test, Shutting Down...")
        cmnd("ST;")
    else:
        print("\n  Shutting Down...")
        cmnd("ST;")
        cmnd("MO=0;")
