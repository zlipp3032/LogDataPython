from rigidBodyState_log import Parameter
import time
import numpy as np
import math as m

defaultParams = Parameter()
defaultParams.Ts = 0.05#20Hz
defaultParams.expectedMAVs = 1
defaultParams.kpx = 0.06
defaultParams.kdx = 0.10
defaultParams.kpy = 0.06
defaultParams.kdy = 0.10
defaultParams.kpz = 0.09
defaultParams.kdz = 0.20
defaultParams.targetAltitude = 3 #  unit is in meters
defaultParams.quadMass = 1.5 #2.25 # units in kg
defaultParams.gravity = 9.81 #units in  m/s/s
# Velocity Controller Gains
defaultParams.ku_vel = 3.5
defaultParams.kv_vel = 3.5
defaultParams.kw_vel = 2.5
defaultParams.rollLimit = 0.7845 # (0.7845 rad  = 45 deg) the upper limit for this drone
defaultParams.pitchLimit = 0.7845 # (0.5236 rad = 30 deg) the upper limit for this drone (1.5708 rad = 90 deg = m.pi/2 rad)
defaultParams.throttleLimit = defaultParams.quadMass*defaultParams.gravity - 0 #This is the hover value with a bias adjustment pending experiments
