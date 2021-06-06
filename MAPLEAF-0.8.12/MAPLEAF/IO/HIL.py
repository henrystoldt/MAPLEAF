'''
Hardware in the loop simulation functionality.
Orientation/Position/Velocity info is passed to a hardware control system, which passes control inputs back to the simulator.

Specific implementation - can serve as inspiration, but would have to be modified to work with other avionics systems
'''

import struct
import time

import serial

from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import Vector


class packet:

    def __init__(self, address):

        self.PTByte = 0
        self.address = address
        self.data = []
        self.checkSumUpperByte = 0
        self.checkSumLowerByte = 0

    def writeData(self, writtenData):

        for i in range(len(writtenData)):

            self.data.append(writtenData[i])

    def computeCheckSum(self):

        total = ord('s') + ord('n') + ord('p') + self.PTByte + self.address

        for i in range(len(self.data)):

            total += self.data[i]

        decimalRepresentation = (int)(total/1.0)
        binaryRepresentation = decimalRepresentation.to_bytes(2,'big',signed=False)

        self.checkSumUpperByte = binaryRepresentation[0]
        self.checkSumLowerByte = binaryRepresentation[1]

    def createPTByte(self, hasData, isBatch, numRegisters):

        PTByteValue = 0

        if(hasData == True):

            PTByteValue += 128

        if(isBatch == True):

            PTByteValue += 64

        PTByteValue += numRegisters*4

        self.PTByte = PTByteValue


class HILInterface:

    def __init__(self, quatUpdateRate, posUpdateRate, velUpdateRate, teensyComPort = "Com20", imuComPort = "Com15", teensyBaudrate = 9600, imuBaudrate = 57600):

        self.teensyComs = serial.Serial()
        self.teensyComs.port = teensyComPort
        self.teensyComs.baudrate = teensyBaudrate
        self.teensyComs.timeout = 0.001

        self.imuSpoof = serial.Serial()
        self.imuSpoof.port = imuComPort
        self.imuSpoof.baudrate = imuBaudrate
        self.imuSpoof.timeout = 0.001

        self.quatUpdatePeriod = 1 / quatUpdateRate
        self.posUpdatePeriod = 1 / posUpdateRate
        self.velUpdatePeriod = 1 / velUpdateRate


        self.startTime = time.time() #Gives milliseconds
        self.lastUpdateTime = self.startTime
        self.lastQuaternionUpdateTime = self.startTime
        self.lastPositionUpdateTime = self.startTime
        self.lastVelocityUpdateTime = self.startTime

        self.canardAngles = [0]*4

        self.teensyComs.open()
        self.imuSpoof.open()

    def sendTeensyCommands(self,command):

        self.teensyComs.write(command)

    def getCanardAngles(self):
        #for i in range(4):
            #print(self.canardAngles[i])
        return self.canardAngles

    def requestCanardAngles(self):

        self.teensyComs.write('q'.encode('UTF-8'))
        time.sleep(0.001)
        for i in range(4):
        
            a = self.teensyComs.readline()
            if a == b'': #If there was a timeout
                self.canardAngles[i] = self.canardAngles[i] #If there was a timeout just use the alst iterations angle
                print("Serial Timeout")
            else:
                self.canardAngles[i] = float(a)

        return self.canardAngles

    def setupHIL(self,currentRigidBodyState):

        self.sendTeensyCommands('R'.encode('UTF-8'))

        self.sendIMUData(currentRigidBodyState,10000)

        time.sleep(2)

        self.sendTeensyCommands('L'.encode('UTF-8'))

        self.startTime = time.time()
        self.lastUpdateTime = self.startTime #Seconds
        self.lastQuaternionUpdateTime = self.startTime
        self.lastPositionUpdateTime = self.startTime
        self.lastVelocityUpdateTime = self.startTime

        return

    def performHIL(self, currentRigidBodyState, simTime):
        
        #Send IMU data if the update times have passed
        self.sendIMUData(currentRigidBodyState, simTime)
        
        #This is the delay waiting for the real world to catch up with the simulation
        while simTime > (time.time() - self.startTime):
            #print("Stalling")
            #print('{0:.8f}'.format(time.time()-self.lastUpdateTime))
            pass

        #Send IMU data if the update times have passed
        self.requestCanardAngles()

        self.lastUpdateTime = time.time()

    def sendIMUData(self, currentRigidBodyState, simTime):

        #print("Hello1")
        #print(simTime)
        #print(self.startTime)
        #print(self.lastQuaternionUpdateTime)
        #print(self.quatUpdatePeriod)

        #If the simulation time is greater than the quat update period
        if((simTime + self.startTime - self.lastQuaternionUpdateTime) > self.quatUpdatePeriod):
            #print("Hello2")
            #send quaternion data
            quaternionByteArray = self.getQuaternionByteArray(currentRigidBodyState)

            myPacket = packet(109)
            myPacket.writeData(quaternionByteArray)
            myPacket.createPTByte(True, True, 3)
            self.writePacket(myPacket)
            #print("Sent")

            self.lastQuaternionUpdateTime = simTime+self.startTime

        if((simTime + self.startTime - self.lastPositionUpdateTime) > self.posUpdatePeriod):

            #send position data
            positionByteArray = self.getPositionByteArray(currentRigidBodyState)

            myPacket = packet(117)
            myPacket.writeData(positionByteArray)
            myPacket.createPTByte(True, True, 4)

            self.writePacket(myPacket)

            self.lastPositionUpdateTime = simTime+self.startTime

        if((simTime+self.startTime - self.lastVelocityUpdateTime) > self.velUpdatePeriod):

            #send velocity data
            velocityByteArray = self.getVelocityByteArray(currentRigidBodyState)

            myPacket = packet(121)
            myPacket.writeData(velocityByteArray)
            myPacket.createPTByte(True, True, 4)
            self.writePacket(myPacket)

            self.lastVelocityUpdateTime = simTime+self.startTime

        return

    def convertQuaternionComponent(self, quaternionComponent):

        imuDecimalRepresentation = (int)(quaternionComponent*29789.09091)
        imuBinaryRepresentation = imuDecimalRepresentation.to_bytes(2,'big',signed=True)

        imuUpperByte = imuBinaryRepresentation[0]
        imuLowerByte = imuBinaryRepresentation[1]
        
        return imuUpperByte, imuLowerByte

    def convertPositionComponent(self, positionComponent):

        ba = bytearray(struct.pack("f", positionComponent))

        imuMSB = ba[3]
        imu2MSB = ba[2]
        imu2LSB = ba[1]
        imuLSB = ba[0]

        return imuMSB, imu2MSB, imu2LSB, imuLSB

    def convertVelocityComponent(self, velocityComponent):

        ba = bytearray(struct.pack("f", velocityComponent))

        imuMSB = ba[3]
        imu2MSB = ba[2]
        imu2LSB = ba[1]
        imuLSB = ba[0]

        return imuMSB, imu2MSB, imu2LSB, imuLSB

    def getQuaternionByteArray(self,currentRigidBodyState):

        convertedQuaternion = self.getConvertedQuaternion(currentRigidBodyState)

        random32BitTimeSpoof = (1).to_bytes(4,'big',signed=True)
        ranTuple = (random32BitTimeSpoof[0], random32BitTimeSpoof[1], random32BitTimeSpoof[2], random32BitTimeSpoof[3])

        byteArray = self.convertQuaternionComponent(convertedQuaternion.Q[0])
        byteArray += (self.convertQuaternionComponent(convertedQuaternion.Q[1]))
        byteArray += (self.convertQuaternionComponent(convertedQuaternion.Q[2]))
        byteArray += (self.convertQuaternionComponent(convertedQuaternion.Q[3]))
        byteArray += ranTuple

        return byteArray

    def getPositionByteArray(self,currentRigidBodyState):

        convertedPosition = self.getConvertedPosition(currentRigidBodyState)

        random32BitTimeSpoof = (1).to_bytes(4,'big',signed=True)
        ranTuple = (random32BitTimeSpoof[0], random32BitTimeSpoof[1], random32BitTimeSpoof[2], random32BitTimeSpoof[3])

        byteArray = self.convertPositionComponent(convertedPosition.X)
        byteArray += self.convertPositionComponent(convertedPosition.Y)
        byteArray += self.convertPositionComponent(convertedPosition.Z)
        byteArray += ranTuple

        return byteArray

    def getVelocityByteArray(self,currentRigidBodyState):

        convertedVelocity = self.getConvertedVelocity(currentRigidBodyState)

        random32BitTimeSpoof = (1).to_bytes(4,'big',signed=True)
        ranTuple = (random32BitTimeSpoof[0], random32BitTimeSpoof[1], random32BitTimeSpoof[2], random32BitTimeSpoof[3])

        byteArray = self.convertVelocityComponent(convertedVelocity.X)
        byteArray += self.convertVelocityComponent(convertedVelocity.Y)
        byteArray += self.convertVelocityComponent(convertedVelocity.Z)
        byteArray += ranTuple

        return byteArray

    def getConvertedQuaternion(self, currentRigidBodyState):

        conversionQuaternion = Quaternion(axisOfRotation=Vector(1,0,0),angle=3.141592654)
        convertedQuaternion = currentRigidBodyState.orientation*conversionQuaternion

        return currentRigidBodyState.orientation

    def getConvertedPosition(self,currentRigidBodyState):

        convertedPosition = Vector(currentRigidBodyState.position.X,currentRigidBodyState.position.Y,-currentRigidBodyState.position.Z)

        return convertedPosition

    def getConvertedVelocity(self,currentRigidBodyState):

        convertedVelocity = Vector(currentRigidBodyState.velocity.X,currentRigidBodyState.velocity.Y,-currentRigidBodyState.velocity.Z)

        return convertedVelocity


    def writePacket(self,myPacket):

        myPacket.computeCheckSum()

        self.imuSpoof.write('s'.encode('UTF-8'))
        self.imuSpoof.write('n'.encode('UTF-8'))
        self.imuSpoof.write('p'.encode('UTF-8'))
        self.imuSpoof.write(bytes([myPacket.PTByte]))
        self.imuSpoof.write(bytes([myPacket.address]))
        self.imuSpoof.write(bytes(myPacket.data))
        self.imuSpoof.write(bytes([myPacket.checkSumUpperByte]))
        self.imuSpoof.write(bytes([myPacket.checkSumLowerByte]))

    def __del__(self):

        self.teensyComs.close()
        self.imuSpoof.close()
