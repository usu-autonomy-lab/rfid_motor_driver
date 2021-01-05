#!/usr/bin/python

import serial
import rospy
import glob
# from std_msgs.msg import Float32
from motor_driver.msg import MotionCart
import subprocess
from numpy import pi
from time import sleep
import threading

baudrates = [4800, 9600, 19200, 38400, 57600, 115200, 230400]
baudrate = baudrates[5]  # use baudrates[5] (115200 baud) 
position_data = []
amperage_data = []
stop_threading = False


class Pub:
    publisher = rospy.Publisher("cart", MotionCart, queue_size = 10)

    def __init__(self):
        rospy.init_node("cart")

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

def cmnd(cmnd):
    ser.write(cmnd)
    try:
        data = ser.readline()
    except:
        rospy.logwarn("Error Reading Response: " + cmnd)
        return
    return data

def read_data_thread():
    while not stop_threading:
        try:
            posData = cmnd("PX;")
	    position_data.append(posData)
            ampData = cmnd("VX;")
            # ampData = cmnd("AN[3];")
	    amperage_data.append(ampData)
	except:
	    rospy.logwarn("Failed to Read Position Data")

def publish_data_thread():
    message = MotionCart()
    while not stop_threading and not rospy.is_shutdown():
        if len(position_data) is not 0:
            data1 = position_data.pop()
            message.position = float(data1.split(";")[1])
        if len(amperage_data) is not 0:
            data2 = amperage_data.pop()
            message.amperage = float(data2.split(";")[1])
        node.publisher.publish(message)

def homing():
    print("Beginning Homing Motion...")
    cmnd("mo=1;")
    cmnd("pr=1;")
    cmnd("bg;")
    cmnd("jv=-20000;")
    cmnd("bg;")

    notDone = True
    while notDone and not rospy.is_shutdown():
        data = cmnd("AN[3];").split(";")[1]
        amperage = float(data)
        if abs(amperage) >= 8:
            print("Hit Current Limit, Homing complete.")
            notDone = False

    cmnd("MO=0;")
    sleep(1)
    cmnd("PX=0;")

if __name__ == '__main__':
    # params = gui.gui()  # [motion_time, maxvel, rate]
    """ errors within the following roslaunch file, removed for test...
    subprocess.call(['roslaunch', 'bench_tf2', 'bench_test.launch', 
                    'x:='+str(params[5]), 'y:='+str(params[6]),
                    'z:='+str(params[7]), 'angle_rad:='+str(params[8] * 180 / pi)])
    """
    node = Pub()
    rate = rospy.Rate(300)

    port = check_ports()
    ser = serial.Serial(port, baudrate, timeout = 0.0015)

    #######################################
    ### Motion Configurations
    #######################################
    maxVelocity = "100" # default: 30, millimeters per second
    maxAcceleration = "600000"  # default: 600000, 600000 max
    motion_time = 300 # duration of motion (seconds)
    MR_mode = "2"  # default: 2, point to point repetative motion
    MR_delay = "1"  # default: 0, delay after finished cycle 
    MR_first = "10000"  # default: steps forwards
    MR_second = "-10000"  # default: steps backwards
    toPrintDetails = True
    STEPS_IN_MM = 75.1879699
    maxStepsPerSec = str(int(maxVelocity) * STEPS_IN_MM)
    
    #######################################
    ### Set Safety Configurations
    #######################################
    cmnd("MO=0;")
    resultMaxAccel = cmnd("AC="+maxAcceleration+";")
    resultMaxDecel = cmnd("DC="+maxAcceleration+";")
    resultMaxVel = cmnd("VH[2]="+maxStepsPerSec+";")
    #######################################

    if toPrintDetails:
	print("Connection Details: ")
        print("Port: " + ser.portstr)
        print("Baudrate: " + str(baudrate))

    if "yes" == raw_input("Run homing procedure? (yes/no): "):
        homing()

    if int(maxAcceleration) > 600000:
        rospy.logwarn("Max Acceleration Exceeds 600,000. Risks Overvoltage to Controller. Do not Proceed.")

    gotInput = False
    while not gotInput:
        test_selection = raw_input("Select Motion Type\n1: Direct Command\n2: Point to Point Motion\n3: Exit\n")
        if test_selection is "1" or "2" or "3":
            gotInput=True

    if test_selection is "1":
        print("Entered Direct Command, Enter 'exit()' to exit OR rm() to enter repetative motion")
        while test_selection is "1":
            userCommand = raw_input(">> ")
            if userCommand == "exit()":
                test_selection = "3"
            elif userCommand == "pub()":
                thread
	    elif userCommand == "rm()":
		test_selection = "2"
            else:
                print("<< " + cmnd(userCommand))
    if test_selection is "2":
        try:
            print("Configuring Motion Controller...")
            cmnd("MO=1;")
            resultMode = cmnd("MR[1]="+MR_mode+";")
            resultDelay = cmnd("MR[2]="+MR_delay+";")
            resultFirst = cmnd("MR[3]="+MR_first+";")
            resultSecond = cmnd("MR[4]="+MR_second+";")
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
            print("Duration:  "+str(motion_time)+" seconds")
            # print("----------------------------------------\n")

        if raw_input("System Configured, Begin? (yes/no): ") != "yes":
            exit()
        print("Send Commands to Motor...")

        data = cmnd("MO=1;")
        if data == "MO=0;;":
            rospy.logfatal("  Motor failed to initialize:  " + data)
            exit()
        print("Beginning Motion...")
        sleep(1)
        cmnd("MO=1;")   
        cmnd("BG;")
    
	t1 = threading.Thread(target=read_data_thread)
    	t2 = threading.Thread(target=publish_data_thread)

	t1.start()
	t2.start()

	sleep(motion_time)

	stop_threading = True

	t1.join()
	t2.join()

	if not rospy.is_shutdown():
	    print("End of Test, Shutting Down...")
	    cmnd("ST;")
        else:
	    print("Forced Exit, Shutting Down...")
	    cmnd("ST;")
	    cmnd("MO=0;")
    else:
        print("Shutting Down...")
        cmnd("ST;")
        cmnd("MO=0;")

