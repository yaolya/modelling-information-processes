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
q0 = 0
maxTime = 10
t = 0
jIdx = 1

if (IS_GUI):
    physicsClient = p.connect(p.GUI)
else:
    physicsClient = p.connect(p.DIRECT)

p.setGravity(0, 0, -g)
bodyId = p.loadURDF("./triple_pendulum.urdf")


p.changeDynamics(bodyUniqueId=bodyId,
                linkIndex=jIdx,
                linearDamping=0)

p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        targetPosition = q0,
                        controlMode = p.POSITION_CONTROL)
for _ in range(1000):
    p.stepSimulation()

q0_fact = p.getJointState(bodyId, jIdx)[0]
print(f'q0 error: {q0 - q0_fact}')
pos0 = [q0_fact, 0]

log_time = [t]
log_pos = [q0_fact]

p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.VELOCITY_CONTROL,
                        targetVelocity = 0,
                        force = 0)
while t <= maxTime:
    p.stepSimulation()
    pos = p.getJointState(bodyId, jIdx)[0]
    t += dt

    p.setJointMotorControl2(bodyIndex = bodyId,
                        jointIndex = jIdx,
                        controlMode = p.TORQUE_CONTROL,
                        force = 0.1)

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