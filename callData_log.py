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
from pymavlink import mavutil

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
        # Arm and Takeoff SOLO and initiate leader as home location
        #self.arm_and_takeoff()
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True
        self.rigidBodyState.leader = self.vehicle.location.global_frame
        
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
        time.sleep(5) # Wait five seconds before sending takeoff command
        while True:
            print 'Altitude:" ', self.vehicle.location.global_relative_frame.alt
            #print self.vehicle.mode
            #print self.rigidBodyState.parameters.targetAltitude
            if self.vehicle.location.global_relative_frame.alt>=self.rigidBodyState.parameters.targetAltitude*0.95:
                print "Reached Target Altitude"
                break
        time.sleep(10) #Wait ten seconds before entering PD control algorithm
        self.rigidBodyState.leader.lat = self.vehicle.location.global_frame.lat
        self.rigidBodyState.leader.lon = self.vehicle.location.global_frame.lon
        self.rigidBodyState.leader.alt = self.vehicle.location.global_frame.alt

        
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
        leadlat = self.rigidBodyState.leader.lat*m.pi/180
        leadlon = self.rigidBodyState.leader.lon*m.pi/180
        self.rigidBodyState.position.x = r*m.cos(lat)*m.cos(lon)
        self.rigidBodyState.position.y = r*m.cos(lat)*m.sin(lon)
        leaderX = r*m.cos(leadlat)*m.cos(leadlon) + 4.000 
        leaderY = r*m.cos(leadlat)*m.sin(leadlon)
        leaderZ = self.rigidBodyState.leader.alt
        self.rigidBodyState.leader.gx = leaderX
        self.rigidBodyState.leader.gy = leaderY
        self.rigidBodyState.leader.gz = leaderZ
        print self.rigidBodyState.leader.gz
        self.rigidBodyState.command.ux = self.rigidBodyState.parameters.kpx*(leaderX-self.rigidBodyState.position.x) + self.rigidBodyState.parameters.kdx*(0 - self.rigidBodyState.velocity.vx)
        self.rigidBodyState.command.uy = self.rigidBodyState.parameters.kpy*(leaderY-self.rigidBodyState.position.y) + self.rigidBodyState.parameters.kdy*(0 - self.rigidBodyState.velocity.vy)
        self.rigidBodyState.command.uz = self.rigidBodyState.parameters.kpz*(leaderZ-self.rigidBodyState.position.z) + self.rigidBodyState.parameters.kdz*(0 - self.rigidBodyState.velocity.vz)
        #print self.rigidBodyState.leader
        #print self.rigidBodyState.command
        self.velocityEstimate()
        
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


    def velocityEstimate(self):
        self.rigidBodyState.command.vel_est_x = self.rigidBodyState.previousState.velPrev_x + 0.5*self.rigidBodyState.previousState.accPrev_x*self.rigidBodyState.parameters.Ts + 0.5*self.rigidBodyState.command.ux*self.rigidBodyState.parameters.Ts
        self.rigidBodyState.command.vel_est_y = self.rigidBodyState.previousState.velPrev_y + 0.5*self.rigidBodyState.previousState.accPrev_y*self.rigidBodyState.parameters.Ts + 0.5*self.rigidBodyState.command.uy*self.rigidBodyState.parameters.Ts
        self.rigidBodyState.command.vel_est_z = self.rigidBodyState.previousState.velPrev_z + 0.5*self.rigidBodyState.previousState.accPrev_z*self.rigidBodyState.parameters.Ts + 0.5*self.rigidBodyState.command.uz*self.rigidBodyState.parameters.Ts
        self.rigidBodyState.previousState.velPrev_x = self.rigidBodyState.command.vel_est_x
        self.rigidBodyState.previousState.velPrev_y = self.rigidBodyState.command.vel_est_y
        self.rigidBodyState.previousState.velPrev_z = self.rigidBodyState.command.vel_est_z
        self.rigidBodyState.previousState.accPrev_x = self.rigidBodyState.command.ux
        self.rigidBodyState.previousState.accPrev_y = self.rigidBodyState.command.uy
        self.rigidBodyState.previousState.accPrev_z = self.rigidBodyState.command.uz
        self.send_ned_velocity(self.rigidBodyState.command.vel_est_x,self.rigidBodyState.command.vel_est_y,self.rigidBodyState.command.vel_est_z,1)
        
    # Velocity commands with respect to home location directionally
    # Be sure tp set up the home location and know your bearings; update the table below before you fly
    # velocity_x > 0 => fly North
    # velocity_x < 0 => fly South
    # velocity_y > 0 => fly East
    # velocity_y < 0 => fly West
    # velocity_z < 0 => ascend
    # velocity_z > 0 => descend
    ###
    def send_ned_velocity(self,velocity_x, velocity_y, velocity_z, duration):
        #    """
        #    Move vehicle in direction based on specified velocity vectors.
        #    """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode( #msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        # send command to vehicle on 1 Hz cycle
        for x in range(0,duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(self.rigidBodyState.parameters.Ts)




        
