import sys
from rigidBodyState_log import *
from datetime import datetime



	
def vsToCSV(vs):
#	print vs.ID
#	print vs.command
	out=''
	out+=str(vs.ID+1) + ','
#	if(vs.isFlocking):
#		out+= '1,'
#	else:
#		out+= '0,'
#   # out+=str(vs.test.xx)+','
#  #  out+=str(vs.test.yy)+','
# #   out+=str(vs.test.zz)+','
##    out+=str(vs.test.qq)+','
	out+=str(vs.test.roll)+','
	out+=str(vs.test.pitch)+','
	out+=str(vs.test.throttle)+','
	out+=str(vs.test.yaw)+','
        out+=str(vs.velocity.vx)+','
	out+=str(vs.velocity.vy)+','
	out+=str(vs.velocity.vz)+','
        out+=str(vs.attitude.roll)+','
        out+=str(vs.attitude.pitch)+','
        out+=str(vs.attitude.yaw)+','
        out+=str(vs.position.x)+','
        out+=str(vs.position.y)+','
        out+=str(vs.position.z)+','
        #out+=str(vs.leader.gx),','
        #out+=str(vs.leader.gy),','
        #out+=str(vs.leader.gz),','
        try:
                out+=str(vs.command.ux)+','
                out+=str(vs.command.uy)+','
                out+=str(vs.command.uz)+','
                out+=str(vs.command.vel_est_x)+','
                out+=str(vs.command.vel_est_y)+','
                out+=str(vs.command.vel_est_z)+','
        except (KeyboardInterrupt, SystemExit):
                raise
	#print (vs.position.keys['long])
#	out+=str(vs.position)+','
#	out+=str(vs.position.lon)+','
#	out+=str(vs.position.alt)+','
	#print (vs.velocity)
#	out+=str(vs.velocity[0])+','
#	out+=str(vs.velocity[1])+','
#	out+=str(vs.velocity[2])+','
#		out+=' ,'
	
#	try:
#		out+=str((vs.timeout.peerLastRX[1]-epoch) . total_seconds())+','
#	except: 
#		out+=' ,'
#	try: 
#		out+=str((vs.timeout.peerLastRX[2]-epoch) . total_seconds())+','
#	except:
#		out+=' ,'
#	try:
#		out+=str((vs.timeout.peerLastRX[3]-epoch) . total_seconds())+','
#	except:
#		out+=' ,'

#	try:
#		out+=str(vs.abortReason) + ','
#	except:
#		out+=' ,'
#	try:
#		out+=str(vs.command.headingRate)+','
#		out+=str(vs.command.climbRate)+','
#		out+=str(vs.command.airSpeed)+','
#		out+=str(vs.command.thetaD)+','
#		out+=str(vs.command.accAltError)+','
#	except (KeyboardInterrupt, SystemExit):
#		raise
#	except:
#		out+=' , , , , ,'
#	out+=str(vs.channels['1']) +', '+ str(vs.channels['2']) +','+ str(vs.channels['3'])+','+str(vs.channels['5'])+','+str(vs.channels['6'])+','
#	out+=str(vs.servoOut['1'])+', ' +str(vs.servoOut['2'])+', ' + str(vs.servoOut['3'])
	return out
	
def msgToCSVHeaders():
	str = "Time,ID,roll,pitch,yaw,lat,lon,alt,latSpd,lngSpd,altSpeed,lastRXGnd,lastRX1,lastRX2,lastRX3, headingRateCmd, climbRateCmd, airSpeedCmd,ch1,ch2,ch3,ch5,ch6"
	return str
def vsToCSVHeaders():
	str = "ID,isFlocking,roll,pitch,yaw,rollspeed,pitchspeed,yawspeed,lat,lon,alt,latSpd,lngSpd,altSpeed,airspeed,heading,headingRate,thetaDDotApprox,lastRXGnd,lastRX1,lastRX2,lastRX3,abortReason, headingRateCmd, climbRateCmd, airSpeedCmd,thetaD,accAltError,ch1,ch2,ch3,ch5,ch6,servo1,servo2,servo3"
	return str
