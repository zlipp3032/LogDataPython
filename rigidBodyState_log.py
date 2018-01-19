from recordtype import recordtype

Timeout = recordtype('Timeout',['localTimeoutTime','GCSLastRx',('peerLastRx',{})],default = None)
Command = recordtype('Command',['Roll','Pitch','Throttle','Yaw'], default = None)

Position = recordtype('Position',['x','y','z'], default=None)
Velocity = recordtype('Velocity',['vx','vy','vz'], default = None)

Parameter = recordtype('Parameter',['Ts','peerTimeout','expectedMAVs'], default = None)

Test = recordtype('Test',['xx','yy','zz','qq'], default = None)

RigidBodyState = recordtype('RigidBodyState', [('startTime', None),'ID',('test',Test()),'time','attitude',('position',Position()),('velocity',Velocity()),('heading',0.0),('command',Command()),('parameters',Parameter()),('timeout',Timeout())], default = None)

Message = recordtype('Message', 'type,sendTime,content', default = None)

