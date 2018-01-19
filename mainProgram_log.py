from dronekit import connect, VehicleMode
import sys
import time
import argparse
import socket
import os
import Queue
import threading
#import receiveClass_log,loggingClass_log, callData_log
import loggingClass_log, callData_log
from defaultParams_log import *
from rigidBodyState_log import *
import numpy as np
from datetime import datetime

#Define the Agent on the network
localIP = '192.168.0.7'
Port = 5001
myAddr = (localIP,Port)
bufferLength = 1000
expectedMAVs = 1
logPath = '/Users/zlipp3032/Documents/MastersThesisUAS/SOLO/Code/ComTests/Control/OldVersions/LogDataScheme/junk'
#broadcastIP = '192.168.0.255' #Will be necessary when we introduce multiple agents
#transmitAddr = (broadcastIP,Port)
#d = 500

startTime = datetime.now()

#Create Message Queues
#receiveQueue = Queue.Queue()
logQueue = Queue.Queue()

#receiveThread = receiveClass_log.Receiver(receiveQueue,localIP,Port,bufferLength)
logThread = loggingClass_log.Logging(logQueue,logPath,expectedMAVs,startTime) 
controlThread = callData_log.Control(logQueue,startTime,localIP,defaultParams) # IF you end up receiving data, you will need to add 'receiveQueue' to the items input into this class

threads = []
#threads.append(receiveThread)
threads.append(logThread)
threads.append(controlThread)

#receiveThread.start()
logThread.start()
controlThread.start()

def hasLiveThreads(threads):
    return True in [t.isAlive() for t in threads]

while hasLiveThreads(threads):
    try:
        [t.join(1) for t in threads
        if t is not None and t.isAlive()]

    except KeyboardInterrupt:
        print "Killing Threads"
        for t in threads:
            t.stop()

print "Exiting mainProgram"
