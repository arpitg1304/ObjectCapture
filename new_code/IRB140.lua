function sysCall_init()
	torques = {0,0,0}
	joints={-1,-1,-1}
    for i=1,3,1 do
        joints[i]=sim.getObjectHandle('IRB140_joint'..i)
    end
    --print(joints[3])
    pub = simROS.advertise('/sensorABB', 'sensor_msgs/JointState')--'sensor_msgs/JointState')
    simROS.publisherTreatUInt8ArrayAsString(pub) -- idk if this is needed
    sub = simROS.subscribe('/torques','sensor_msgs/JointState', 'callback')
    simROS.subscriberTreatUInt8ArrayAsString(sub) -- idk if this is needed
    joint_angles={0,0,0}
    joint_velocities={0,0,0}
end

function callback(msg)
	torques = msg.effort
	print('inside callback')
end

function sysCall_sensing()
	for i=1,3,1 do
		status,joint_velocities[i]=sim.getObjectFloatParameter(joints[i],2012)
		joint_angles[i]=sim.getJointPosition(joints[i])
	end
	--for i=1,#joint_velocities do
      --  joint_angles[#joint_angles+1] = joint_velocities[i]
    --end
    print(joint_angles)
	print(joint_velocities)
    d = {}
    d['velocity'] = joint_velocities
    d['position'] = joint_angles
	simROS.publish(pub,d)
end

function sysCall_actuation()
    print(torques)
    for i=1,3,1 do
    	local epsilon = 1e-2
    	if math.abs(torques[i])>epsilon then
            sim.setJointTargetVelocity(joints[i],9999)
            sim.setJointForce(joints[i],torques[i])
        else
        sim.setJointTargetVelocity(joints[i],0)
        sim.setJointForce(joints[i],9999)
        end
	end
end
