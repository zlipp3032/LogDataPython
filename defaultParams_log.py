from rigidBodyState_log import Parameter
import time
import numpy as np

defaultParams = Parameter()
defaultParams.Ts = 0.01#20Hz
defaultParams.expectedMAVs = 1
defaultParams.kpx = 0.3
defaultParams.kdx = 0.8
defaultParams.kpy = 0.3
defaultParams.kdy = 0.8
defaultParams.kpz = 0.4
defaultParams.kdz = 0.8
defaultParams.targetAltitude = 3 #  unit is in meters
