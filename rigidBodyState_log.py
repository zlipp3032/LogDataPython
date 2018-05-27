from recordtype import recordtype

Timeout = recordtype('Timeout',['localTimeoutTime','GCSLastRx',('peerLastRx',{})],default = None)
Command = recordtype('Command',['Roll','Pitch','Throttle','Yaw','ux','uy','uz','vel_est_x','vel_est_y','vel_est_z'], default = None)

PreviousState = recordtype('PreviousState',[('velPrev_x',0),('velPrev_y',0),('velPrev_z',0),('accPrev_x',0),('accPrev_y',0),('accPrev_z',0)], default = None)

Position = recordtype('Position',['x','y','z'], default=None)
Velocity = recordtype('Velocity',['vx','vy','vz'], default = None)
Leader = recordtype('Leader',['lat','lon','alt','gx','gy','gz','gvx','gvy','gvz'], default = None)

Parameter = recordtype('Parameter',['Ts','peerTimeout','expectedMAVs','kpx','kdx','kpy','kdy','kpz','kdz','targetAltitude','quadMass','gravity','ku_vel','kv_vel','kw_vel','rollLimit','pitchLimit','throttleLimit','stoppingDistance','desiredSpeed','isTakeoff'], default = None)

AttThrust = recordtype('AttThrust',['roll','pitch','throttle','yaw'], default = None)

RigidBodyState = recordtype('RigidBodyState', [('startTime', None),'ID',('test',AttThrust()),'time','attitude',('position',Position()),('velocity',Velocity()),('heading',0.0),('command',Command()),('parameters',Parameter()),('timeout',Timeout()),('leader',Leader()),('previousState',PreviousState())], default = None)

Message = recordtype('Message', 'type,sendTime,content', default = None)

