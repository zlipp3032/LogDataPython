from dronekit import connect, VehicleMode, Vehicle
import time
import logging
from rigidBodyState_log import *
import os
import Queue
import threading
import math as m
import numpy as np
from datetime import datetime, timedelta

logging.basicConfig(level=logging.WARNING)

class Control(threading.Thread):
    def __init__(self,logQueue,startTime,localIP,defaultParams,vehicle): # IF you end up receiving data, you will need to add 'receiveQueue' to the items input into this class
        threading.Thread.__init__(self)
        self.isRunning=True
        self.logQueue = logQueue
#        self.receiveQueue = receiveQueue
#        self.RigidBodies = {}
        self.rigidBodyState = RigidBodyState()
        self.rigidBodyState.startTime = datetime.now()
        self.counter = 0
        self.stoprequest = threading.Event()
        self.startTime = startTime
        self.rigidBodyState.ID = 1 #localIP BESURE TO UPDATE THIS WHEN IT COMES TIME FOR MULTIAGENT TESTING!!!!!
        self.rigidBodyState.parameters = defaultParams
        self.vehicle = vehicle
        # Arm and Takeoff SOLO
        self.arm_and_takeoff()
        
    def stop(self):
        self.stoprequest.set()
        print "Stop Flag Set - Control"

    def run(self):
        while(not self.stoprequest.is_set()):
##########################################################
##########################################################
#This section is only required if data is received on net#
##########################################################
            #while(not self.receiveQueue.empty()):
            #    try:
            #        msg = self.receiveQueue.get(False)
            #        self.updateGlobalStatewithData(msg)
            #        self.getRigidBodyState(msg)
            #        self.receiveQueue.task_done()
            #    except Queue.Empty:
            #        break #no more messages
##########################################################
##########################################################
            self.getData()
            self.computeControl()
##            self.pushStatetoTxQueue()
            self.pushStatetoLoggingQueue()
            time.sleep(self.rigidBodyState.parameters.Ts)
        self.stop()
        print "Control Stopped"

        

##########################################################
##########################################################
#This section is only required if data is received on net#
##########################################################
#    def updateGlobalStatewithData(self,msg):
#        self.decodeMessage(msg)

#    def decodeMessage(self,msg):
#        if(msg.content.ID>0):
#            ID = int(msg.content.ID)
#            self.RigidBodies[ID] = msg.content
#           self.rigidBodyState.position = msg.content.position
#            self.rigidBodyState.timeout.peerLastRx[ID] = datetime.now()



#    def getRigidBodyState(self,msg):
#        self.rigidBodyState.timeout.peerLastRx[self.rigidBodyState.ID] = datetime.now() #This might cause issues the way this is currently set up...need to fix...
##        self.rigidBodyState.position = self.RigidBodies[ID]
#        self.rigidBodyState.position = msg.content.position
#        self.rigidBodyState.velocity = msg.content.velocity
##        print self.rigidBodyState.position
#        self.rigidBodyState.time = datetime.now()
#        self.counter+=1
##       self.rigidBodyState.velocity = BackEuler()
##########################################################
##########################################################


    def arm_and_takeoff(self):
        print 'Basic Prearm Checks'
        while not self.vehicle.is_armable:
            print 'Waiting for vehicle to initialize'
            time.sleep(1)
        print 'Arming Motors'
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True
        self.vehicle.simple_takeoff(self.rigidBodyState.parameters.targetAltitude)
        while True:
            print 'Altitude:" ', self.vehicle.location.global_relative_frame.alt
            #print self.vehicle.mode
            #print self.rigidBodyState.parameters.targetAltitude
            if self.vehicle.location.global_relative_frame.alt>=self.rigidBodyState.parameters.targetAltitude*0.95:
                print "Reached Target Altitude"
                break
        time.sleep(10)
        self.rigidBodyState.leader.gx = self.vehicle.location.global_frame.lat*m.pi/180
        self.rigidBodyState.leader.gy = self.vehicle.location.global_frame.lon*m.pi/180
        self.rigidBodyState.leader.gz = self.vehicle.location.global_relative_frame.alt

        
    def getData(self):
        #x = m.sin(self.counter)
        #y = m.cos(self.counter)
        #z = self.counter^3
        #q = m.e
        ROLL = self.vehicle.channels['1']
        PITCH =  self.vehicle.channels['2']
        THROTTLE = self.vehicle.channels['3']
        YAW = self.vehicle.channels['4']
        self.counter += 1
        self.rigidBodyState.test.roll = ROLL
        self.rigidBodyState.test.pitch = PITCH
        self.rigidBodyState.test.throttle = THROTTLE
        self.rigidBodyState.test.yaw = YAW
        self.rigidBodyState.velocity.vx = self.vehicle.velocity[0]
        self.rigidBodyState.velocity.vy = self.vehicle.velocity[1]
        self.rigidBodyState.velocity.vz = self.vehicle.velocity[2]
        self.rigidBodyState.attitude = self.vehicle.attitude
        #self.rigidBodyState.position.x = self.vehicle.location.global_frame.lat
        #self.rigidBodyState.position.y = self.vehicle.location.global_frame.lon
        self.rigidBodyState.position.z = self.vehicle.location.global_frame.alt 

    def computeControl(self):
        # Convert the x,y positions to meters...
        rEarth = 6378137 #m
        r = rEarth + self.rigidBodyState.position.z
        lat = self.vehicle.location.global_frame.lat*m.pi/180
        lon = self.vehicle.location.global_frame.lon*m.pi/180
        self.rigidBodyState.position.x = r*m.cos(lat)*m.cos(lon)
        self.rigidBodyState.position.y = r*m.cos(lat)*m.sin(lon)
        leaderX = r*m.cos(self.rigidBodyState.leader.gx)*m.cos(self.rigidBodyState.leader.gy) + 4 
        leaderY = r*m.cos(self.rigidBodyState.leader.gx)*m.sin(self.rigidBodyState.leader.gy)
        leaderZ = self.rigidBodyState.leader.gz
        self.rigidBodyState.command.ux = self.rigidBodyState.parameters.kpx*(leaderX-self.rigidBodyState.position.x) + self.rigidBodyState.parameters.kdx*(0 - self.rigidBodyState.velocity.vx)
        self.rigidBodyState.command.uy = self.rigidBodyState.parameters.kpy*(leaderY-self.rigidBodyState.position.y) + self.rigidBodyState.parameters.kdy*(0 - self.rigidBodyState.velocity.vy)
        self.rigidBodyState.command.uz = self.rigidBodyState.parameters.kpz*(leaderZ-self.rigidBodyState.position.z) + self.rigidBodyState.parameters.kdz*(0 - self.rigidBodyState.velocity.vz)
        print self.rigidBodyState.leader
        #print self.rigidBodyState.command
        
##  def pushStatetoTxQueue(self):
##      msg = Message()
##      msg.type = "UAV"
##      msg.sendTime = datetime.now()
##      msg.content = self.rigidBodyState
##        self.transmitQueue.put(msg)
##      return msg

    def pushStatetoLoggingQueue(self):
        msg = Message()
        msg.type = "UAV_LOG"
        msg.sendTime = time.time()
        msg.content = {}
        msg.content['Data'] = self.rigidBodyState
#        msg.content['thisBodyState'] = self.rigidBodyState
#        msg.content['RigidBodies'] = self.RigidBodies
##        print msg.content
        self.logQueue.put(msg)




        
