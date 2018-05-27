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
        #self.customTakeoff()
        self.prepTakeoff()
        #self.vehicle.mode = VehicleMode("STABILIZE")
        #self.vehicle.armed = True
        #self.getLeaderData()
        #time.sleep(5)
        
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
            if(self.rigidBodyState.parameters.isTakeoff):
                if(not self.checkAbort()):
                    self.computePDControl()
            else:
                if(not self.vehicle.location.global_relative_frame.alt>=self.rigidBodyState.parameters.targetAltitude*0.95):
                    if(not self.checkAbort()):
                        desDest = self.rigidBodyState.position.z - self.rigidBodyState.leader.alt
                        self.computeTakeoffVelocity(desDest)                  
                    #self.computeControl()
                else:
                    print "Reached Target Altitude"
                    self.rigidBodyState.parameters.isTakeoff = True
                    self.getLeaderData()
##            self.pushStatetoTxQueue()
            self.pushStatetoLoggingQueue()
            time.sleep(self.rigidBodyState.parameters.Ts)
        self.releaseControl()
        self.stop()
        print "Control Stopped"


    def prepTakeoff(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        print 'Basic Prearm Checks'
        #insert Checks
        print 'Arming Motors'
        self.vehicle.channels.overrides = {'3':1000}
        time.sleep(2)
        #self.vehicle.armed = True
        #self.getData()
        self.getLeaderData()
        time.sleep(2)
        self.rigidBodyState.leader.alt = -self.rigidBodyState.parameters.targetAltitude

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


    def computeTakeoffVelocity(self,desDest):    
        if(abs(desDest) >= self.rigidBodyState.parameters.stoppingDistance):
            self.rigidBodyState.leader.gvz = (self.rigidBodyState.parameters.desiredSpeed*desDest)/abs(desDest)
            print self.rigidBodyState.leader
            if(not self.checkAbort()):
                self.computePDControl()
                print "Taking Off"
        else:
            self.rigidBodyState.leader.gvz = (self.rigidBodyState.parameters.desiredSpeed*desDest)/self.rigidBodyState.parameters.stoppingDistance
            if(not self.checkAbort()):
                self.computePDControl()
                print "Approaching Target Altitude"

    def customTakeoff(self):
        self.vehicle.mode = VehicleMode('STABILIZE')
        print 'Basic Prearm Checks'
        #insert Checks
        print 'Arming Motors'
        self.vehicle.channels.overrides = {'3':1000}
        time.sleep(2)
        self.vehicle.armed = True
        #self.getData()
        self.getLeaderData()
        time.sleep(5)
        self.rigidBodyState.leader.alt = -self.rigidBodyState.parameters.targetAltitude
        #print self.rigidBodyState.leader
        while True:
            self.getData()
            if(not self.vehicle.location.global_relative_frame.alt>=self.rigidBodyState.parameters.targetAltitude*0.95):
               if(not self.checkAbort()):
                   desDest = self.rigidBodyState.position.z - self.rigidBodyState.leader.alt
                   self.computeTakeoffVelocity(desDest)                  
                   #self.computeControl()
            else:
                print "Reached Target Altitude"
                self.rigidBodyState.parameters.isTakeoff = True
                break
##            self.pushStatetoTxQueue()
            self.pushStatetoLoggingQueue()
            time.sleep(self.rigidBodyState.parameters.Ts)
        self.getLeaderData()
        
        
    def arm_and_takeoff(self):
        print 'Basic Prearm Checks'
        while not self.vehicle.is_armable:
            print 'Waiting for vehicle to initialize'
            time.sleep(1)
        print 'Arming Motors'
        self.vehicle.mode = VehicleMode('GUIDED')
        self.vehicle.armed = True
        time.sleep(5)  # Wait five seconds before sending takeoff command
        self.vehicle.simple_takeoff(self.rigidBodyState.parameters.targetAltitude)
        while True:
            #print 'Altitude:" ', self.vehicle.location.global_relative_frame.alt
            #print self.vehicle.mode
            #print self.rigidBodyState.parameters.targetAltitude
            if self.vehicle.location.global_relative_frame.alt>=self.rigidBodyState.parameters.targetAltitude*0.95:
                print "Reached Target Altitude"
                break
        time.sleep(10) #Wait ten seconds before entering PD control algorithm
        #self.rigidBodyState.leader.lat = self.vehicle.location.global_frame.lat
        #self.rigidBodyState.leader.lon = self.vehicle.location.global_frame.lon
        #self.rigidBodyState.leader.alt = self.vehicle.location.global_frame.alt
        self.getLeaderData()

    def getLeaderData(self):
        self.rigidBodyState.leader.lat = self.vehicle.location.global_frame.lat
        self.rigidBodyState.leader.lon = self.vehicle.location.global_frame.lon
        self.rigidBodyState.leader.alt = -self.vehicle.location.global_relative_frame.alt
        self.rigidBodyState.leader.gvx = 0
        self.rigidBodyState.leader.gvy = 0
        self.rigidBodyState.leader.gvz = 0
        print self.rigidBodyState.leader

        
    def checkAbort(self):
        # Check for proper flght mode
        if(not (self.vehicle.mode == 'STABILIZE')):
            #self.rigidBodyState.RCLatch = True
            #print "Not in Proper Control Mode"
            #print "Current Mode: %s" % self.vehicle.mode
            self.releaseControl()
            return True
        return False
        
    def getData(self):
        #x = m.sin(self.counter)
        #y = m.cos(self.counter)
        #z = self.counter^3
        #q = m.e
        YAW = self.vehicle.channels['4']
        self.counter += 1
        self.rigidBodyState.test.yaw = YAW
        self.rigidBodyState.velocity.vx = self.vehicle.velocity[0]
        self.rigidBodyState.velocity.vy = self.vehicle.velocity[1]
        self.rigidBodyState.velocity.vz = -self.vehicle.velocity[2]
        self.rigidBodyState.attitude = self.vehicle.attitude
        # Convert the x,y positions to meters...
        #rEarth = 6378137 #m
        self.rigidBodyState.position.z = -self.vehicle.location.global_relative_frame.alt 
        #r = rEarth - self.rigidBodyState.position.z
        lat = self.vehicle.location.global_frame.lat#*m.pi/180
        lon = self.vehicle.location.global_frame.lon#*m.pi/180
        qi_gps = np.matrix([lat, lon])
        qkl_gps = np.matrix([self.rigidBodyState.leader.lat, self.rigidBodyState.leader.lon])
        dq = self.getRelPos(qi_gps,qkl_gps)
        self.rigidBodyState.position.y = dq[0,0]
        self.rigidBodyState.position.x = dq[0,1]
        #self.rigidBodyState.position.x = -r*m.cos(lat)*m.cos(lon)
        #self.rigidBodyState.position.y = r*m.cos(lat)*m.sin(lon)
        
    def computePDControl(self):
        self.rigidBodyState.command.uz = self.rigidBodyState.parameters.kpz*(self.rigidBodyState.leader.alt-self.rigidBodyState.position.z) + self.rigidBodyState.parameters.kdz*(self.rigidBodyState.leader.gvz - self.rigidBodyState.velocity.vz)
        # Everything Above this might be wrong....
        self.rigidBodyState.command.ux = self.rigidBodyState.parameters.kpx*self.rigidBodyState.position.x + self.rigidBodyState.parameters.kdx*(self.rigidBodyState.leader.gvx - self.rigidBodyState.velocity.vx)
        self.rigidBodyState.command.uy = self.rigidBodyState.parameters.kpy*self.rigidBodyState.position.y + self.rigidBodyState.parameters.kdy*(self.rigidBodyState.leader.gvy - self.rigidBodyState.velocity.vy)
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
        self.rigidBodyState.command.vel_est_x = self.rigidBodyState.previousState.velPrev_x + 0.5*(self.rigidBodyState.previousState.accPrev_x + self.rigidBodyState.command.ux)*self.rigidBodyState.parameters.Ts
        self.rigidBodyState.command.vel_est_y = self.rigidBodyState.previousState.velPrev_y + 0.5*(self.rigidBodyState.previousState.accPrev_y + self.rigidBodyState.command.uy)*self.rigidBodyState.parameters.Ts
        self.rigidBodyState.command.vel_est_z = self.rigidBodyState.previousState.velPrev_z + 0.5*(self.rigidBodyState.previousState.accPrev_z + self.rigidBodyState.command.uz)*self.rigidBodyState.parameters.Ts
        self.rigidBodyState.previousState.velPrev_x = self.rigidBodyState.command.vel_est_x
        self.rigidBodyState.previousState.velPrev_y = self.rigidBodyState.command.vel_est_y
        self.rigidBodyState.previousState.velPrev_z = self.rigidBodyState.command.vel_est_z
        self.rigidBodyState.previousState.accPrev_x = self.rigidBodyState.command.ux 
        self.rigidBodyState.previousState.accPrev_y = self.rigidBodyState.command.uy
        self.rigidBodyState.previousState.accPrev_z = self.rigidBodyState.command.uz
        #self.send_ned_velocity(self.rigidBodyState.command.vel_est_x,self.rigidBodyState.command.vel_est_y,self.rigidBodyState.command.vel_est_z,1)
        #self.send_ned_velocity(self.rigidBodyState.command.vel_est_x,0,0,1)
        self.computeAttitudeThrustCommands()

    def computeAttitudeThrustCommands(self):
        self.rigidBodyState.test.throttle = self.rigidBodyState.parameters.quadMass*(self.rigidBodyState.parameters.gravity - self.rigidBodyState.command.uz + self.rigidBodyState.parameters.kw_vel*(self.rigidBodyState.velocity.vz - self.rigidBodyState.command.vel_est_z))/(np.cos(self.rigidBodyState.attitude.roll)*np.cos(self.rigidBodyState.attitude.pitch))
        self.rigidBodyState.test.pitch = np.arctan(((self.rigidBodyState.command.ux-self.rigidBodyState.parameters.ku_vel*(self.rigidBodyState.velocity.vx - self.rigidBodyState.command.vel_est_x))*np.cos(self.rigidBodyState.attitude.yaw) + (self.rigidBodyState.command.uy-self.rigidBodyState.parameters.kv_vel*(self.rigidBodyState.velocity.vy - self.rigidBodyState.command.vel_est_y))*np.sin(self.rigidBodyState.attitude.yaw))/(-self.rigidBodyState.parameters.gravity + self.rigidBodyState.command.uz - self.rigidBodyState.parameters.kw_vel*(self.rigidBodyState.velocity.vz - self.rigidBodyState.command.vel_est_z)))
        self.rigidBodyState.test.roll = np.arctan(((self.rigidBodyState.command.ux-self.rigidBodyState.parameters.ku_vel*(self.rigidBodyState.velocity.vx - self.rigidBodyState.command.vel_est_x))*np.cos(self.rigidBodyState.test.pitch)*np.sin(self.rigidBodyState.attitude.yaw) - (self.rigidBodyState.command.uy-self.rigidBodyState.parameters.kv_vel*(self.rigidBodyState.velocity.vy - self.rigidBodyState.command.vel_est_y))*np.cos(self.rigidBodyState.test.pitch)*np.cos(self.rigidBodyState.attitude.yaw))/(-self.rigidBodyState.parameters.gravity + self.rigidBodyState.command.uz - self.rigidBodyState.parameters.kw_vel*(self.rigidBodyState.velocity.vz - self.rigidBodyState.command.vel_est_z)))
        #print self.rigidBodyState.test
        self.scaleAndSendControl()

    def saturate(self, value, minimum, maximum):
        out = max(value,minimum)
        out = min(out,maximum)
        return out

    def getRelPos(self,pos1,pos2): #returns the x y delta position of p2-p1 with x being longitude (east positive)
        c = 40074784 # from https://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
        dx = (pos2[0,1]-pos1[0,1]) * c * m.cos(m.radians( (pos1[0,0]+pos2[0,0])/ 2))/360
        dy = (pos2[0,0]-pos1[0,0]) * c /360	
        return np.matrix([dx, dy])


    def releaseControl(self):
        self.vehicle.channels.overrides = {}
        print self.vehicle.channels.overrides
        print "Channels Cleared"
    
    def scaleAndSendControl(self):
        #Scale the compute control values to match the format used in vehicle.channel.overrides{}
        ROLL =  1500 + (500/self.rigidBodyState.parameters.rollLimit)*self.rigidBodyState.test.roll
        PITCH = 1500 + (500/self.rigidBodyState.parameters.pitchLimit)*self.rigidBodyState.test.pitch
        THROTTLE = 972 + 48.484*self.rigidBodyState.test.throttle + 1.3241*self.rigidBodyState.test.throttle*self.rigidBodyState.test.throttle#1000 + 32.254*self.rigidBodyState.test.throttle - 0.257*self.rigidBodyState.test.throttle*self.rigidBodyState.test.throttle#
        YAW = self.rigidBodyState.attitude.yaw
        # Saturate to keep commands in range of input values
        self.rigidBodyState.command.Roll = self.saturate(ROLL,1000,2000)
        self.rigidBodyState.command.Pitch = self.saturate(PITCH,1000,2000)
        self.rigidBodyState.command.Throttle = self.saturate(THROTTLE,1000,2000)
        self.rigidBodyState.command.Yaw = self.saturate(YAW,1000,2000)
        self.vehicle.channels.overrides = {'1': self.rigidBodyState.command.Roll,'2': self.rigidBodyState.command.Pitch,'3': self.rigidBodyState.command.Throttle}
        print self.vehicle.channels.overrides
        #print self.rigidBodSytate.leader
        
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
        self.vehicle.send_mavlink(msg)
        #for x in range(0,duration):
            #self.vehicle.send_mavlink(msg)
            #time.sleep(self.rigidBodyState.parameters.Ts)

            


        
