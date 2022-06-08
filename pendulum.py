import pybullet as p
import time
import math
import numpy as np

IS_GUI = False
OWN_REG = True
dt = 1/240
q0 = 0
pos_d = math.pi/2
trajTime = 4
maxTime = 5
t = 0
jIdx = 1
g = 10
L = 0.8
m = 1
kf = 0.1
a = g/L
c = m*L*L
b = kf/c

K = np.array([[1600,  120]])
(mx,mv,vx) = (-52.921769241021074, -10.326402285952112, -128.06248474865563)
if (IS_GUI):
    physicsClient = p.connect(p.GUI)
else:
    physicsClient = p.connect(p.DIRECT)

p.setGravity(0, 0, -g)
bodyId = p.loadURDF("./pendulum.urdf")

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
print(f'q0 fact: {q0_fact}')
print(f'q0 error: {q0 - q0_fact}')
pos0 = [q0_fact, 0]

kp = K[0, 0]
kv = K[0, 1]
ki = 80


def feedback_lin(pos, vel, pos_d, vel_d, ff_acc):
    u_nonlin = a*math.sin(pos)+b*vel
    u_lin = ff_acc - kp*(pos-pos_d) - kv*(vel-vel_d)
    return c*(u_nonlin + u_lin)


def degree_5_param(pos_0, pos_d, T, t):
    a3 = 10/T**3
    a4 = -15/T**4
    a5 = 6/T**5
    s = a3*t**3 + a4*t**4 + a5*t**5
    diff = pos_d-pos_0
    return ((pos_0 + s*diff), diff*(3*a3*t**2 + 4*a4*t**3 + 5*a5*t**4), diff*(6*a3*t + 12*a4*t**2 + 20*a5*t**3)) if (t <= T) else (pos_d, 0, 0)

def degree_7_param(pos_0, pos_d, T, t):
    a4 = 35/T**4
    a5 = -84/T**5
    a6 = 70/T**6
    a7 = -20/T**7
    s = a4*t**4 + a5*t**5 + a6*t**6 + a7*t**7
    diff = pos_d-pos_0
    return ((pos_0 + s*diff), diff*(4*a4*t**3 + 5*a5*t**4 + 6*a6*t**5 + 7*a7*t**6), diff*(12*a4*t**2 + 20*a5*t**3 + 30*a6*t**4 + 42*a7*t**5), diff*(24*a4*t + 60*a5*t**2 + 120*a6*t**3 + 210*a7*t**4)) if (t <= T) else (pos_d, 0, 0, 0)


log_time = [t]
log_pos = [q0_fact]
log_pos_d = [q0_fact]
log_vel= [0]
log_vel_d = [0]
log_acc = [0]
log_acc_d = [0]
log_x_3 = [0]
log_x_3_d = [0]
log_ctrl = []
u = 0

if (OWN_REG):
    p.setJointMotorControl2(bodyIndex = bodyId,
                            jointIndex = jIdx,
                            controlMode = p.VELOCITY_CONTROL,
                            targetVelocity = 0,
                            force = 0)
e_int = 0
prev_vel = 0
prev_acc = 0

while t <= maxTime:
    pos = p.getJointState(bodyId, jIdx)[0]
    vel = p.getJointState(bodyId, jIdx)[1]
    acc = (vel-prev_vel)/dt

    (curr_pos_d, curr_vel_d, curr_acc_d, curr_x_3_d) = degree_7_param(q0_fact, pos_d, trajTime, t)
    u = feedback_lin(pos, vel, curr_pos_d, curr_vel_d, curr_acc_d)

    if (OWN_REG):
        p.setJointMotorControl2(bodyIndex = bodyId,
                            jointIndex = jIdx,
                            controlMode = p.TORQUE_CONTROL,
                            force = u)
    else:
        p.setJointMotorControl2(bodyIndex = bodyId,
                            jointIndex = jIdx,
                            targetPosition = curr_pos_d,
                            controlMode = p.POSITION_CONTROL)

    p.stepSimulation()
    t += dt
    log_pos.append(pos)
    log_vel.append(vel)
    log_acc.append(acc)
    log_x_3.append((acc-prev_acc)/dt)
    log_pos_d.append(curr_pos_d)
    log_vel_d.append(curr_vel_d)
    log_acc_d.append(curr_acc_d)
    log_x_3_d.append(curr_x_3_d)
    prev_vel = vel
    prev_acc = acc
    log_ctrl.append(u)
    log_time.append(t)
    if (IS_GUI):
        time.sleep(dt)
p.disconnect()

import matplotlib.pyplot as plt

# position plot
plt.subplot(5,1,1)
plt.plot(log_time, log_pos, label='sim_pos')
plt.plot(log_time, log_pos_d, label='ref_pos')
plt.grid(True)
plt.legend()

# velocity plot
plt.subplot(5,1,2)
plt.plot(log_time, log_vel, label='sim_vel')
plt.plot(log_time, log_vel_d, label='ref_vel')
plt.grid(True)
plt.legend()

# acceleration plot
plt.subplot(5,1,3)
plt.plot(log_time, log_acc, label='sim_acc')
plt.plot(log_time, log_acc_d, label='ref_acc')
plt.grid(True)
plt.legend()

# x3 plot
plt.subplot(5,1,4)
plt.plot(log_time, log_x_3, label='sim_3')
plt.plot(log_time, log_x_3_d, label='ref_3')
plt.grid(True)
plt.legend()

# control plot
plt.subplot(5,1,5)
plt.plot(log_time[0:-1], log_ctrl, label='control')
plt.grid(True)
plt.legend()

plt.show()
