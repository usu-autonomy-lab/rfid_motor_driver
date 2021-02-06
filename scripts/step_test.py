#!/usr/bin/env python3

import mercury
import serial
import sys
from datetime import datetime
from time import sleep

'''
Paramters:
    Ports, Current Limits, Velocity Limits, etc.
Setup:
    Configure serial connection to Antennas
    Configure serial connection to Motor Controller
    TO ADD: Configure connection to USRP

Safety:
    Set limit parameters on Elmo Driver

Begin Test:
    1) Initialize the motion state to 30cm distance (from home location)
    2) Move in 0.5mm increments
    3) Collect 1 sample from USRP reader
    4) Wait 0.1 second
    5) Collect 1 sample from the COTS reader
    6) Return to step 2, until we reach 80cm (i.e. 80 iterations)
'''

def reader_callback(*arguments):
    with open("./output/test_data.txt", 'a') as f:
        for i in range(len(arguments)):
            if i is not len(arguments) - 1:
                print(arguments[i], end=", ", file=f)
            else:
                print(arguments[i], end="\n", file=f)
    
def cmnd(serial, cmnd):
    serial.write(str.encode(cmnd))
    try:
        data = serial.readline()
    except:
        print("Error reading response: " + cmnd)
        return
    return data.decode("utf-8")

if __name__ == '__main__':
    # Paramters
    start_pos = 300 # units of millimeters (mm)
    iterations = 3  #80
    step_size = 10  #0.5  # units of millimeters (mm)
    # Setup
    debug = True
    antenna_port = "/dev/" + "ttyACM1"
    motor_port = "/dev/" + "ttyACM0"
    baudrate = 115200
    time_test = datetime.now().strftime("%d-%m-%Y_%H-%M")
    #output_file = "./output/test_data" + time_test + ".txt"
    # change in reader_callback(), line 29
    output_file = "./output/test_data"
    # Safety
    maxVel = 300  # units of millimeter per second (mm/s)
    maxAccel = "600000"
    STEPS_PER_MM = 75.1879699
    maxStepsPerSec = str(maxVel * STEPS_PER_MM)

    # Setup: Motor Driver
    if debug:
        input("Connect Motor Driver at port: "+motor_port+"? (Press any key)")

    motor_driver = serial.Serial(motor_port, baudrate, timeout = 0.002)

    if debug:
        input("Set Parameters of Motor Driver. Proceed? (Press any key)")

    cmnd(motor_driver,"MO=0;") 
    cmnd(motor_driver,"AC="+maxAccel+";")
    cmnd(motor_driver,"DC="+maxAccel+";")
    cmnd(motor_driver,"VH[2]="+maxStepsPerSec+";")

    # Run Homing Function
    if "yes" == input("Run homing procedure? (yes): "):
        cmnd(motor_driver, "ST;")
        cmnd(motor_driver, "MO=1;")
        cmnd(motor_driver, "PR=1;")
        cmnd(motor_driver, "BG;")
        cmnd(motor_driver, "JV=-20000;")
        cmnd(motor_driver, "BG;")
        
        notDone = True
        while notDone:
            command = cmnd(motor_driver, "AN[3];")
            data = command.split(";")[1]
            amperage = float(data)
            if abs(amperage) >= 10:
                print("Homing Complete, Hit Current Limit (10 Amps)")
                notDone = False
        cmnd(motor_driver,"MO=0;")
        sleep(0.25)
        cmnd(motor_driver, "PX=0;")

    # Prepare for Test
    if debug:
        input("Move to starting position? (Press any key)")
        
    cmnd(motor_driver,"MO=1;")
    cmnd(motor_driver,"PR=" + str(start_pos * STEPS_PER_MM) + ";")
    cmnd(motor_driver,"BG;")
    sleep(1.5)  # wait until achieves start position
    cmnd(motor_driver,"MO=0;")

    motor_driver.close()

    # Setup: Antenna Reader
    if debug:
        input("Connect Reader at port: "+antenna_port+"? (Press any key)")

    antenna_reader = mercury.Reader("tmr://"+antenna_port, baudrate=baudrate)
    antenna_reader.set_read_plan(antennas=[2], protocol="GEN2")
    antenna_reader.set_region("NA")
    antenna_reader.set_hop_table([910000])
    
    if debug:
        input("Begin Test? (Press any key)")

    with open(output_file, 'w') as f:
        print("Beginning Test -- " + time_test, file=f)

        # Begin Test
        motor_driver.open()
        print(cmnd(motor_driver, "MO=1;"))
        print(cmnd(motor_driver,"PR=" + str(step_size * STEPS_PER_MM) + ";"))
        motor_driver.close()
        sleep(3)

        for i in range(iterations):
            print("i: " + str(i))
            # read with COTS
            antenna_reader.start_reading(lambda tag: reader_callback(tag.epc, tag.rssi, tag.phase, tag.frequency, tag.timestamp))
            sleep(0.1)
            antenna_reader.stop_reading()
            sleep(1)
            # WILL IMPLEMENT READING FROM USRP
            motor_driver.open()
            print(cmnd(motor_driver,"BG;"))
            sleep(1)
            motor_driver.close()
            sleep(3)

        # End of the Test
        print("End of Test -- " + time_test, file=f)

