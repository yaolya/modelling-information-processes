import pybullet as p
import time

IS_GUI = True
dt = 1/240
g = 10
L = 0.8
m = 1
kf = 1
a = g/L
b = kf/(m*L*L)
q0 = [0, 1, 3]
maxTime = 10
t = 0
q0idx = 0

if (IS_GUI):
    physicsClient = p.connect(p.GUI)
else:
    physicsClient = p.connect(p.DIRECT)

p.setGravity(0, 0, -g)
bodyId = p.loadURDF("./triple_pendulum.urdf")

jIdx = p.getNumJoints(bodyId)
forces = [0.] * jIdx
velocities = [0.] * jIdx
jointIndices = []

for i in range(jIdx):
    jointIndices.append(i)
    joint_info = p.getJointInfo(bodyId, i)
    if joint_info[2] is p.JOINT_REVOLUTE:
        print("revolute ", i, " ", joint_info)
        p.changeDynamics(bodyUniqueId=bodyId,
                        linkIndex=i,
                        linearDamping=0)

        p.setJointMotorControl2(bodyIndex = bodyId,
                                jointIndex = i,
                                targetPosition = q0[q0idx],
                                controlMode = p.POSITION_CONTROL)
        q0idx += 1

for _ in range(1000):
    p.stepSimulation()

q0_fact = p.getJointState(bodyId, 1)[0]
print(f'q0 error: {q0[0] - q0_fact}')
pos0 = [q0_fact, 0]

log_time = [t]
log_pos = [q0_fact]

p.setJointMotorControlArray(bodyIndex = bodyId,
                        jointIndices = jointIndices,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocities = velocities,
                        forces = forces)
while t <= maxTime:
    p.stepSimulation()
    pos = p.getJointState(bodyId, 1)[0]
    t += dt

    p.setJointMotorControlArray(bodyIndex = bodyId,
                        jointIndices = jointIndices,
                        controlMode = p.TORQUE_CONTROL,
                        forces = forces)

    log_pos.append(pos)
    log_time.append(t)
    if (IS_GUI):
        time.sleep(dt)
p.disconnect()

import matplotlib.pyplot as plt
plt.plot(log_time, log_pos, label='sim')
plt.grid(True)
plt.legend()
plt.show()