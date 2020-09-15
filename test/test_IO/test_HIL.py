#Created by: Declan Quinn
#May 2019

#To run tests:
#In this file: [test_Vector.py]
#In all files in the current directory: [python -m unittest discover]
#Add [-v] for verbose output (displays names of all test functions)

import time
import unittest

import serial

import MAPLEAF.IO.HIL as HIL
from MAPLEAF.IO.HIL import packet
from MAPLEAF.Motion import AngularVelocity
from MAPLEAF.Motion import Quaternion
from MAPLEAF.Motion import RigidBodyState
from MAPLEAF.Motion import Vector


class testHILInterface(unittest.TestCase):

    def setUp(self):
        try:
            self.interface = HIL.HILInterface(100,20,20)
            self.isNotConnectedToTeensy = True
        except serial.serialutil.SerialException:
            self.isNotConnectedToTeensy = True
            self.skipTest("Teensy not connected, skipping HIL tests")

    def test_getConvertedQuaternion(self):

        state3 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(1, 0, 0), 0.34906585), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        convertedQuaternion = self.interface.getConvertedQuaternion(state3)

        self.assertAlmostEqual(convertedQuaternion.Q[0],0.9848077530468392,4)
        self.assertAlmostEqual(convertedQuaternion.Q[1],0.17364817747052724,4)
        self.assertAlmostEqual(convertedQuaternion.Q[2],0)
        self.assertAlmostEqual(convertedQuaternion.Q[3],0)

    def test_getConvertedPosition(self):

        state3 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(1, 0, 0), 0.34906585), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        convertedPosition = self.interface.getConvertedPosition(state3)

        self.assertAlmostEqual(convertedPosition.X,state3.position.X)
        self.assertAlmostEqual(convertedPosition.Y,state3.position.Y)
        self.assertAlmostEqual(convertedPosition.Z,-state3.position.Z)

    def test_getConvertedVelocity(self):

        state3 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(1, 0, 0), 0.34906585), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        convertedVelocity = self.interface.getConvertedVelocity(state3)

        self.assertAlmostEqual(convertedVelocity.X,state3.velocity.X)
        self.assertAlmostEqual(convertedVelocity.Y,state3.velocity.Y)
        self.assertAlmostEqual(convertedVelocity.Z,-state3.velocity.Z)

    def test_convertQuaternionComponent(self):

        quaternionComponent1 = 1
        quaternionComponent2 = -1
        correctConvertQuaternionData1 = (116,93)
        correctConvertQuaternionData2 = (139,163)

        self.assertEqual(self.interface.convertQuaternionComponent(quaternionComponent1),correctConvertQuaternionData1)
        self.assertEqual(self.interface.convertQuaternionComponent(quaternionComponent2),correctConvertQuaternionData2)

    def test_convertPositionComponent(self):

        positionComponent1 = 250
        positionComponent2 = -250
        correctConvertPositionData1 = (67,122,0,0)
        correctConvertPositionData2 = (195,122,0,0)

        self.assertEqual(self.interface.convertPositionComponent(positionComponent1),correctConvertPositionData1)
        self.assertEqual(self.interface.convertPositionComponent(positionComponent2),correctConvertPositionData2)

    def test_convertVelocityComponent(self):

        velocityComponent1 = 250
        velocityComponent2 = -250
        correctConvertVelocityData1 = (67,122,0,0)
        correctConvertVelocityData2 = (195,122,0,0)

        self.assertEqual(self.interface.convertVelocityComponent(velocityComponent1),correctConvertVelocityData1)
        self.assertEqual(self.interface.convertVelocityComponent(velocityComponent2),correctConvertVelocityData2)

    def test_getQuaternionByteArray(self):

        state1 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        state2 = RigidBodyState(Vector(0, 0, 200), Vector(0, 0, 200), Quaternion(Vector(0, 0, 1), 180), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        self.assertEqual(self.interface.getQuaternionByteArray(state1),(116,93,0,0,0,0,0,0,0,0,0,1))
        self.assertEqual(self.interface.getQuaternionByteArray(state2),(203,221,0,0,0,0,104,7,0,0,0,1))

    def test_getPositionByteArray(self):

        state1 = RigidBodyState(Vector(0, 0, 250), Vector(0, 0, 250), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        state2 = RigidBodyState(Vector(0, 0, -250), Vector(0, 0, -250), Quaternion(Vector(0, 0, 0), 180), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        self.assertEqual(self.interface.getPositionByteArray(state1),(0,0,0,0,0,0,0,0,195,122,0,0,0,0,0,1))
        self.assertEqual(self.interface.getPositionByteArray(state2),(0,0,0,0,0,0,0,0,67,122,0,0,0,0,0,1))

    def test_getVelocityByteArray(self):

        state1 = RigidBodyState(Vector(0, 0, 250), Vector(0, 0, 250), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))
        state2 = RigidBodyState(Vector(0, 0, -250), Vector(0, 0, -250), Quaternion(Vector(0, 0, 0), 180), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        self.assertEqual(self.interface.getVelocityByteArray(state1),(0,0,0,0,0,0,0,0,195,122,0,0,0,0,0,1))
        self.assertEqual(self.interface.getVelocityByteArray(state2),(0,0,0,0,0,0,0,0,67,122,0,0,0,0,0,1))

    def test_createPTByte(self):

        myPacket = packet(109)

        correctPTByteValue1 = 204
        correctPTByteValue2 = 208

        myPacket.createPTByte(True,True,3)
        self.assertEqual(myPacket.PTByte,correctPTByteValue1)

        myPacket.createPTByte(True,True,4)
        self.assertEqual(myPacket.PTByte,correctPTByteValue2)

    def test_computeCheckSum(self):

        correctCheckSumUpper = 3
        correctCheckSumLower = 204
        
        state1 = RigidBodyState(Vector(0, 0, 250), Vector(0, 0, 250), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        self.interface.getPositionByteArray(state1)

        myPacket = packet(109)

        myPacket.createPTByte(True,True,4)

        packetData = self.interface.getPositionByteArray(state1)

        myPacket.writeData(packetData)

        myPacket.computeCheckSum()
        
        self.assertEqual(myPacket.checkSumUpperByte,correctCheckSumUpper)
        self.assertEqual(myPacket.checkSumLowerByte,correctCheckSumLower)

    def test_requestCanardAngles(self):

        time.sleep(0.005)

        self.interface.requestCanardAngles()

        self.assertEqual(self.interface.canardAngles[0],0)
        self.assertEqual(self.interface.canardAngles[1],0)
        self.assertEqual(self.interface.canardAngles[2],0)
        self.assertEqual(self.interface.canardAngles[3],0)

    def test_writeData(self):

        state1 = RigidBodyState(Vector(0, 0, 250), Vector(0, 0, 250), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        data = self.interface.getQuaternionByteArray(state1)

        myPacket = packet(109)
        myPacket.writeData(data)

        self.assertEqual(myPacket.data[0],116)
        self.assertEqual(myPacket.data[1],93)
        self.assertEqual(myPacket.data[2],0)
        self.assertEqual(myPacket.data[3],0)
        self.assertEqual(myPacket.data[4],0)
        self.assertEqual(myPacket.data[5],0)
        self.assertEqual(myPacket.data[6],0)
        self.assertEqual(myPacket.data[7],0)
        self.assertEqual(myPacket.data[8],0)
        self.assertEqual(myPacket.data[9],0)
        self.assertEqual(myPacket.data[10],0)
        self.assertEqual(myPacket.data[11],1)

    def test_writePacket(self):
     
        state1 = RigidBodyState(Vector(0, 0, 250), Vector(0, 0, 250), Quaternion(Vector(0, 0, 1), 0), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        data = self.interface.getQuaternionByteArray(state1)
        
        myPacket = packet(109)
        myPacket.createPTByte(True,True,3)
        myPacket.writeData(data)

        self.interface.writePacket(myPacket)

        data = self.interface.getPositionByteArray(state1)

        myPacket = packet(117)
        myPacket.createPTByte(True,True,4)
        myPacket.writeData(data)

        self.interface.writePacket(myPacket)

    def test_sendIMUData(self):

        simTime = time.time() + 1000

        state1 = RigidBodyState(Vector(0, 0, 250), Vector(0, 0, 250), Quaternion(axisOfRotation=Vector(0, 0, 1), angle=3.141592654), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        self.interface.sendIMUData(state1, simTime)

    def test_fullInterface(self):

        state = RigidBodyState(Vector(0, 0, 0), Vector(0, 0, 0), Quaternion(axisOfRotation=Vector(0, 0, 1), angle=0), AngularVelocity(rotationVector=Vector(0, 0, 0)))

        self.interface.setupHIL(state)

        #self.interface.teensyComs.close()

        print("go")

        #time.sleep(5)

        for i in range(1000):

            print("Working")

            state = RigidBodyState(Vector(0, 0, i), Vector(0, 0, i), Quaternion(axisOfRotation=Vector(0, 0, 1), angle=i*0.0003141592654), AngularVelocity(rotationVector=Vector(0, 0, 0)))

            self.interface.sendIMUData(state, i*5)

            time.sleep(0.001)

            self.interface.requestCanardAngles()

            print(self.interface.getCanardAngles())

if __name__ == '__main__':
    unittest.main()
