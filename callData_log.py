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
    def __init__(self,logQueue,startTime,localIP,defaultParams): # IF you end up receiving data, you will need to add 'receiveQueue' to the items input into this class
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



    def getData(self):
         x = m.sin(self.counter)
         y = m.cos(self.counter)
         z = self.counter^3
         q = m.e
         self.counter += 1
         self.rigidBodyState.test.xx = x
         self.rigidBodyState.test.zz = z
         self.rigidBodyState.test.yy = y
         self.rigidBodyState.test.qq = q

         

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




        
