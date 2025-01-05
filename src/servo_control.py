#!/usr/bin/env python3

import os
import numpy as np 
import msvcrt
from SCSCtrl.scservo_sdk import *  

class ServoControl:
    # Control table address
    ADDR_SCS_TORQUE_ENABLE     = 40
    ADDR_STS_GOAL_ACC          = 41
    ADDR_STS_GOAL_POSITION     = 42
    ADDR_STS_GOAL_SPEED        = 46
    # ADDR_STS_PRESENT_POSITION  = 56
    ADDR_SCS_PRESENT_POSITION  = 56

    # Default setting
    SCS1_ID                     = 1                 # SCServo#1 ID : 1
    SCS2_ID                     = 2                 # SCServo#1 ID : 2
    BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
    DEVICENAME                  = '/dev/ttyTHS1'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

    SCS_MINIMUM_POSITION_VALUE  = 100               # SCServo will rotate between this value
    SCS_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the SCServo would not move when the position value is out of movable range. Check e-manual about the range of the SCServo you use.)
    SCS_MOVING_STATUS_THRESHOLD = 20                # SCServo moving status threshold
    SCS_MOVING_SPEED            = 0                 # SCServo moving speed
    SCS_MOVING_ACC              = 0                 # SCServo moving acc
    protocol_end                = 1                 # SCServo bit end(STS/SMS=0, SCS=1)

    def __init__(self):
        self.linkageLenA = 90
        self.linkageLenB = 160

        self.servoNumCtrl = [0,1]
        self.servoDirection = [1,-1]

        self.servoInputRange = 850
        self.servoAngleRange = 180

        self.servoInit = [None, 512, 512, 512, 512, 512]

        self.nowPos = [None, 512, 512, 512, 512, 512]
        self.nextPos = [None, 512, 512, 512, 512, 512]
        self.speedBuffer = [None, 512, 512, 512, 512, 512]

        self.xMax = 150
        self.xMin = 90

        self.yMax = 170
        self.yMix = -170

        if os.name == 'nt':
            print('nt')
            def getch():
                return msvcrt.getch().decode()
        else:
            import sys, tty, termios
            fd = sys.stdin.fileno()
            #old_settings = termios.tcgetattr(fd)
            def getch():
                try:
                    tty.setraw(sys.stdin.fileno())
                    ch = sys.stdin.read(1)
                finally:
                    #termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                    pass
                return ch

        index = 0
        scs_goal_position = [self.SCS_MINIMUM_POSITION_VALUE, self.SCS_MAXIMUM_POSITION_VALUE]         # Goal position


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize PacketHandler instance
        # Get methods and members of Protocol
        self.packetHandler = PacketHandler(self.protocol_end)

        # Initialize GroupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_STS_GOAL_POSITION, 2)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()


    def syncCtrl(self, ID_List, Speed_List, Goal_List):
        positionList = []

        for i in range(0, len(ID_List)):
            try:
                scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID_List[i], self.ADDR_STS_GOAL_SPEED, Speed_List[i])
            except:
                time.sleep(0.1)
                scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(self.portHandler, ID_List[i], self.ADDR_STS_GOAL_SPEED, Speed_List[i])

            positionBuffer = [SCS_LOBYTE(Goal_List[i]), SCS_HIBYTE(Goal_List[i])]
            positionList.append(positionBuffer)
        
        for i in range(0, len(ID_List)):
            scs_addparam_result = self.groupSyncWrite.addParam(ID_List[i], positionList[i])
        
        scs_comm_result = self.groupSyncWrite.txPacket()
        self.groupSyncWrite.clearParam()


    def infoSingleGet(self, SCID):
        scs_present_position_speed, scs_comm_result, scs_error = packetHandler.read4ByteTxRx(portHandler, SCID, ADDR_SCS_PRESENT_POSITION)
        # if scs_comm_result != COMM_SUCCESS:
        #     print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        # elif scs_error != 0:
        #     print("%s" % packetHandler.getRxPacketError(scs_error))

        scs_present_position = SCS_LOWORD(scs_present_position_speed)
        # scs_present_speed = SCS_HIWORD(scs_present_position_speed)
        # print("[ID:%03d] PresPos:%03d PresSpd:%03d"%(SCID, scs_present_position, SCS_TOHOST(scs_present_speed, 15)))

        return scs_present_position


    def portClose(self):
        self.portHandler.closePort()

    def servoAngleCtrl(self, ServoNum, AngleInput, DirectionDebug, SpeedInput):
        offsetGenOut = self.servoInit[ServoNum] + int((self.servoInputRange/self.servoAngleRange)*AngleInput*DirectionDebug)
        self.syncCtrl([ServoNum], [SpeedInput], [offsetGenOut])
        return offsetGenOut


    def returnOffset(self, ServoNum, AngleInput, DirectionDebug):
        offsetGenOut = self.servoInit[ServoNum] + int((self.servoInputRange/self.servoAngleRange)*AngleInput*DirectionDebug)
        return offsetGenOut

    def nowPosUpdate(self, servoNumInput):
        scs_present_position_speed, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(self.portHandler, servoNumInput, self.ADDR_SCS_PRESENT_POSITION)
        scs_present_position = SCS_LOWORD(scs_present_position_speed)
        
        self.nowPos[servoNumInput] = scs_present_position
        # print(scs_present_position)
        return scs_present_position

    def servoStop(self, servoNum):
        scs_present_position_speed, scs_comm_result, scs_error = self.packetHandler.read4ByteTxRx(self.portHandler, servoNum, self.ADDR_SCS_PRESENT_POSITION)
        if scs_comm_result != COMM_SUCCESS:
            print(self.packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print(self.packetHandler.getRxPacketError(scs_error))

        scs_present_position = SCS_LOWORD(scs_present_position_speed)
        scs_present_speed = SCS_HIWORD(scs_present_position_speed)
        # print("[ID:%03d] GoalPos:%03d PresPos:%03d PresSpd:%03d" 
        #       % (SCS_ID, scs_goal_position[index], scs_present_position, SCS_TOHOST(scs_present_speed, 15)))
        self.syncCtrl([servoNum], [0], [scs_present_position])


    def stopServo(self, servoNumInput):
        try:
            self.servoStop(servoNumInput)
        except:
            time.sleep(0.1)
            self.servoStop(servoNumInput)


