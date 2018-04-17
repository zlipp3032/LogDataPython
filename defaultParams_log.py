from rigidBodyState_log import Parameter
import time
import numpy as np

defaultParams = Parameter()
defaultParams.Ts = 0.01#20Hz
defaultParams.expectedMAVs = 1
defaultParams.kpx = 0.4
defaultParams.kdx = 0.6
defaultParams.kpy = 0.4
defaultParams.kdy = 0.6
defaultParams.kpz = 0.4
defaultParams.kdz = 0.6
defaultParams.targetAltitude = 3 #  unit is in meters
