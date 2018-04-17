from recordtype import recordtype

Timeout = recordtype('Timeout',['localTimeoutTime','GCSLastRx',('peerLastRx',{})],default = None)
Command = recordtype('Command',['Roll','Pitch','Throttle','Yaw','ux','uy','uz'], default = None)

Position = recordtype('Position',['x','y','z'], default=None)
Velocity = recordtype('Velocity',['vx','vy','vz'], default = None)
Leader = recordtype('Leader',['gx','gy','gz'], default = None)

Parameter = recordtype('Parameter',['Ts','peerTimeout','expectedMAVs','kpx','kdx','kpy','kdy','kpz','kdz','targetAltitude'], default = None)

Test = recordtype('Test',['roll','pitch','throttle','yaw'], default = None)

RigidBodyState = recordtype('RigidBodyState', [('startTime', None),'ID',('test',Test()),'time','attitude',('position',Position()),('velocity',Velocity()),('heading',0.0),('command',Command()),('parameters',Parameter()),('timeout',Timeout()),('leader',Leader())], default = None)

Message = recordtype('Message', 'type,sendTime,content', default = None)

