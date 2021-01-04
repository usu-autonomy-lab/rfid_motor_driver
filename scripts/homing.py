#!/usr/bin/python

import serial

def getPort():
    ports = glob.glob('/dev/ttyACM*')
    if(len(ports)==0):
        rospy.logfatal("No serial connection detected.")
        rospy.logfatal("Check if hardware is connected properly.")
        exit(1)
    elif(len(ports) > 1):
        print("Found ports:")
        for x in range(len(ports)):
            print(str(x) + " - " + ports[x])
        while True:
            try:
                port_num = int(raw_input("Please select a valid port number: "))
                if(port_num not in range(len(ports))):
                    continue
                break
            except ValueError:
                continue
        return

def cmnd(cmnd):
    ser.write(cmnd)
    try:
        data = ser.readline()
    except:
        rospy.logwarn("Error Reading Response: " + cmnd)
        return
    return data

if __name__ == '__main__':
    port = getPort()
    ser = serial.Serial(port, baudrate, timeout = 0.0015)

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

    print("Powering On...")
    print("\t" + cmnd("MO=0;"))

    
