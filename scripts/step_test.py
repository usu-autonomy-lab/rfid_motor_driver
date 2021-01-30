#!/usr/bin/env python3

import mercury
import serial

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

if __name__ == '__main__':
    STEPS_PER_MM = 75.1879699
    # Paramters
    start_pos = 300 # units of millimeters (mm)
    iterations = 80
    step_size = 0.5  # units of millimeters (mm)
    maxVel = 300  # units of millimeter per second (mm/s)
    # Setup
    debug = True
    antenna_port = "ttyACM0"
    motor_port = "ttyACM1"
    baudrate = 115200
    maxStepsPerSec = str(maxVel * STEPS_PER_MM)
    # Safety
    maxAccel = "600000"

    # Setup: Antenna Reader
    if debug:
        input("Connect Reader at port: "+antenna_port+"? (Press any key)")

    antenna_reader = mercury.Reader("tmr:///dev/"+antenna_port, baudrate=baudrate)
    antenna_reader.set_read_plan(antennas=[2], protocol="GEN2")
    antenna_reader.set_region("NA")
    antenna_reader.set_hop_table([910000])
    
    # Setup: Motor Driver
    if debug:
        input("Connect Motor Driver at port: "+motor_port+"? (Press any key)")

    motor_driver = serial.Serial(motor_port, baudrate, timeout = 0.002)
    motor_driver.write(str.encode("MO=0;"))
    motor_driver.write(str.encode("AC="+maxAccel+";"))
    motor_driver.write(str.encode("DC="+maxAccel+";"))
    motor_driver.write(str.encode("VH[2]="+maxStepsPerSec+";"))

    if debug:
        input("Serial Connections Initiated. Proceed? (Press any key)")

    if "yes" == input("Run homing procedure? (yes): "):
        motor_driver.write(str.encode("MO=1;"))
        motor_driver.write(str.encode("PR=1;"))
        motor_driver.write(str.encode("BG;"))
        motor_driver.write(str.encode("JV=-20000;"))
        motor_driver.write(str.encode("BG;"))
        
        notDone = True
        while notDone:
            data = motor_driver.write(str.encode("AN[3];")).split(";")[1]
            amperage = float(data)
            if abs(amperage) >= 8:
                print("Homing Complete, Hit Current Limit (8 Amps)")
                notDone = False
        motor_driver.write(str.encode("MO=0;"))
        sleep(0.5)
        motor_driver.write(str.encode())
        motor_driver.write(str.encode())

    # Begin Test
        # read with COTS
        # wait 0.1 s
        # read with USRP
        # move 0.5mm
        # repeat

