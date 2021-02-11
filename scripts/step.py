#!/usr/bin/env python3

import serial
import glob
import sys
from time import sleep

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
                port_num = int(input("Please select a valid port number: "))
                if(port_num not in range(len(ports))):
                    continue
                break
            except ValueError:
                continue
        return ports[port_num]
    else:
        return ports[0]


def cmnd(serial, cmnd):
    serial.write(str.encode(cmnd))
    try:
        data = serial.readline()
    except:
        print("error reading response: " + cmnd)
        return
    return data.decode('utf-8')

def readData(response):
    return float(response.split(";")[1])

def homing(serial):
    cmnd(serial, "ST;")
    cmnd(serial, "MO=1;")
    cmnd(serial, "PR=1;")
    cmnd(serial, "BG;")
    cmnd(serial, "JV=-20000;")
    cmnd(serial, "BG;")

    notDone = True
    while notDone:
        amperage = readData(cmnd(serial, "AN[3];"))
        if abs(amperage) >= 9:
            print("Homing Complete, Hit Current Limit (9 Amps)")
            notDone = False
    cmnd(serial, "MO=0;")
    sleep(0.25)
    cmnd(serial, "PX=0;")



if __name__ == '__main__':
    ## Default Test Parameters
    start = 300  # mm
    step = 1  # mm
    reps = 3  # repititions
    delay = 0.5  # s
    velocity = 100  # mm/s

    ## Advanced Test Parameters
    baudrate = 115200
    STEPS_IN_MM = 75.1879699
    serial_timeout = 0.002  # s
    maxAccel = "600000"
    endlimit = 1230  # mm
    
    ## Get Commandline Paramters
    args = sys.argv
    if len(args) > 1:
        for arg in args:
            if arg is args[0]:
                continue
            try:
                if "start" in arg:
                    arg = arg.split("=")
                    start = float(arg[1])
                elif "step" in arg:
                    arg = arg.split("=")
                    step = float(arg[1])
                elif "reps" in arg:
                    arg = arg.split("=")
                    reps = int(arg[1])
                elif "delay" in arg:
                    arg = arg.split("=")
                    delay = float(arg[1])
                else:
                    print("Unable to parse: " + str(arg) + " skipping...")
            except:
                print("Failed to parse: " + str(arg) + " skipping...")

    ## Configure Test
    endpos = reps*step + start

    if endpos > endlimit:
        print("ERROR: End position exceeds upper limit, " + str(endlimit))
        print("Given: End position is: " + str(endpos) + " mm") 
        print("Exiting...")
        exit()

    maxStepsPerSec = str(velocity * STEPS_IN_MM)
    startpos = str(start * STEPS_IN_MM)
    stepsize = str(step * STEPS_IN_MM)

    ## Connect to Elmo
    port = check_ports()
    ser = serial.Serial(port, baudrate, timeout = serial_timeout)

    ## Set Safety Parameters
    resultMaxAccel = cmnd(ser, "AC="+maxAccel+";")
    resultMaxDecel = cmnd(ser, "DC="+maxAccel+";")
    resultMaxVel = cmnd(ser, "VH[2]="+maxStepsPerSec+";")

    ## Confirm Test Parameters
    print("")
    print("Confirm Test Parameters --")
    print("Baudrate: " + str(baudrate))
    print("    Port: " + port)
    print("--------------------------")
    print("Start Position: " + str(start) + " mm\t[" + startpos + "]")
    print("     Step Size: " + str(step) + " mm\t[" + stepsize + "]")
    print("  End Position: " + str(endpos) + " mm\t[" + str(endpos * STEPS_IN_MM) + "]")
    print("   Repititions: " + str(reps))
    print("         Delay: " + str(delay) + " s")
    print("      Velocity: " + str(velocity) + "\t[" + resultMaxVel + "]")
    print("  Acceleration: " + str(int(int(maxAccel) * 1/STEPS_IN_MM)) + "\t[" + resultMaxAccel + "]")
    print("--------------------------")
    
    # homing function
    input("Run Homing Procedure? (Press any key)")
    homing(ser)
    
    # start
    cmnd(ser, "MO=1;")
    print("Going to start position: " + str(start))
    cmnd(ser, "PR=" + startpos + ";")
    cmnd(ser, "BG;")

    input("Proceed with test? (Press any key)")
    cmnd(ser, "PR=" + stepsize + ";")

    for i in range(reps):
        # record arrival time stamp
        cmnd(ser, "BG;")
        sleep(0.2)
        while readData(cmnd(ser, "MS;")) != float(1):
            pass
        sleep(delay)
        # record departure time stamp

