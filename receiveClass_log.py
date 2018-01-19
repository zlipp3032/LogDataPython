# Not used in the current main example -- can implement by uncommenting necessary code throughout this branch

import socket
from rigidBodyState import *
import collections
import Queue
import threading
from datetime import datetime

class Receiver(threading.Thread):
    def __init__(self,receiveQueue,localIP,Port,bufferLength):
        threading.Thread.__init__(self)
        self.localIP = localIP
        self.IP = '' #Symbolic name meaning local host
        self.Port = Port
        self.UDPTIMEOUT = 10 #value in seconds
        self.bufferLength = bufferLength
        self.localAddr = (self.IP,self.Port)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST, 1)
        self.socket.settimeout(self.UDPTIMEOUT)
        self.socket.bind(self.localAddr)
        self.receiveQueue=receiveQueue
        self.stoprequest = threading.Event()
        self.rigidBodyState = RigidBodyState()

    def stop(self):
        self.stoprequest.set()
        print "Stop flag set - Receive"

    def run(self):
        while( not self.stoprequest.is_set()):
            try:
                self.receiveMessage()
            except Queue.Empty:
                break
        print "Receive Stopped"
        
    def receiveMessage(self):
        try:
            udpData = self.socket.recvfrom(self.bufferLength)
            msg = Message()
            msg.content = RigidBodyState()
            msg.sendTime = datetime.now() #This is acutally the time we received the message from the computer
            msg.content.position = udpData[0].split(',')
            msg.content.ID = 1 #Will designate the first cell as the computer
#            print 'Receive Data'
            #print "udpData: %s" % udpData
#            print "Msg: %s" % msg.content
            self.receiveQueue.put(msg)
            pass
        except socket.error, e:
            if not e.args[0] == 'timed out':
                raise e
            else: print "timeout"
