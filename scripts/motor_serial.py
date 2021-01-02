#!/usr/bin/python

import serial
import rospy
import glob
from std_msgs.msg import Float32
import gui
import subprocess
from numpy import pi
from time import sleep
import threading

baudrates = [4800, 9600, 19200, 38400, 57600, 115200, 230400]
baudrate = baudrates[5]  # use baudrates[5] (115200 baud) 
position_data = []
stop_threading = False


class Pub:
    pos_publisher = rospy.Publisher("position", Float32, queue_size = 10)
    pose = Float32()

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

def cmnd(cmnd):
    ser.write(cmnd)
    try:
        data = ser.readline()
    except:
        rospy.logwarn("Error Reading Response: " + cmnd)
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
            data = position_data.pop()
            position = float(data.split(";")[1])
            node.float32 = position
            node.pos_publisher.publish(position)


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
    ser.timeout = 0.0015

    ### Motion Configurations
    maxVelocity = "5000" # default: 1000, meters per second
    maxAcceleration = "600000"  # default: 600000, 600000 max
    motion_time = 30 # duration of motion (seconds)
    MR_mode = "2"  # default: 2, point to point repetative motion
    MR_delay = "0"  # default: 0, delay after finished cycle 
    MR_first = "10000"  # default: steps forwards
    MR_second = "-10000"  # default: steps backwards
    toPrintDetails = True
    STEPS_IN_METER = 75187.9699
    maxStepsPerSec = int(maxVelocity) * STEPS_IN_METER
    
    ### Set Safety Configurations
    cmnd("MO=0;")
    resultMaxAccel = cmnd("AC="+maxAcceleration+";")
    resultMaxDecel = cmnd("DC="+maxAcceleration+";")
    resultMaxVel = cmnd("VH[2]="+maxVelocity+";")

    if toPrintDetails:
	print("Connection Details: ")
        print("Port: " + ser.portstr)
        print("Baudrate: " + str(baudrate) + "\n")

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
    
	t1 = threading.Thread(target=read_position_thread)
    	t2 = threading.Thread(target=publish_position_thread)

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

