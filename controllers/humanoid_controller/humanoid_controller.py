"""humanoid_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from math import *
import numpy as np
import sys

from motor_sim import *

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

def leg_motor_define():
    motor_L1 = robot.getDevice('Roll-L')
    motor_L2 = robot.getDevice('Yaw-L')
    motor_L3 = robot.getDevice('Pitch-L')
    motor_L4 = robot.getDevice('Knee-L')
    motor_L5 = robot.getDevice('Ankle-Roll-L')
    motor_L6 = robot.getDevice('Ankle-Pitch-L')

    sensor_L1 = robot.getDevice('Roll-L_sensor')
    sensor_L2 = robot.getDevice('Yaw-L_sensor')
    sensor_L3 = robot.getDevice('Pitch-L_sensor')
    sensor_L4 = robot.getDevice('Knee-L_sensor')
    sensor_L5 = robot.getDevice('Ankle-Pitch-L_sensor')
    sensor_L6 = robot.getDevice('Ankle-Roll-L_sensor')

    motor_L1_dir = -1.0
    motor_L2_dir = 1.0
    motor_L3_dir = 1.0
    motor_L4_dir = 1.0
    motor_L5_dir = 1.0
    motor_L6_dir = 1.0

    motor_R1 = robot.getDevice('Roll-R')
    motor_R2 = robot.getDevice('Yaw-R')
    motor_R3 = robot.getDevice('Pitch-R')
    motor_R4 = robot.getDevice('Knee-R')
    motor_R5 = robot.getDevice('Ankle-Roll-R')
    motor_R6 = robot.getDevice('Ankle-Pitch-R')

    sensor_R1 = robot.getDevice('Roll-R_sensor')
    sensor_R2 = robot.getDevice('Yaw-R_sensor')
    sensor_R3 = robot.getDevice('Pitch-R_sensor')
    sensor_R4 = robot.getDevice('Knee-R_sensor')
    sensor_R5 = robot.getDevice('Ankle-Pitch-R_sensor')
    sensor_R6 = robot.getDevice('Ankle-Roll-R_sensor')

    motor_R1_dir = -1.0
    motor_R2_dir = 1.0
    motor_R3_dir = -1.0
    motor_R4_dir = -1.0
    motor_R5_dir = -1.0
    motor_R6_dir = -1.0

    motor_sim_L1 = MotorSim(motor_L1, sensor_L1, motor_L1_dir, timestep)
    motor_sim_L2 = MotorSim(motor_L2, sensor_L2, motor_L2_dir, timestep)
    motor_sim_L3 = MotorSim(motor_L3, sensor_L3, motor_L3_dir, timestep)
    motor_sim_L4 = MotorSim(motor_L4, sensor_L4, motor_L4_dir, timestep)
    motor_sim_L5 = MotorSim(motor_L5, sensor_L5, motor_L5_dir, timestep)
    motor_sim_L6 = MotorSim(motor_L6, sensor_L6, motor_L6_dir, timestep)

    motor_sim_R1 = MotorSim(motor_R1, sensor_R1, motor_R1_dir, timestep)
    motor_sim_R2 = MotorSim(motor_R2, sensor_R2, motor_R2_dir, timestep)
    motor_sim_R3 = MotorSim(motor_R3, sensor_R3, motor_R3_dir, timestep)
    motor_sim_R4 = MotorSim(motor_R4, sensor_R4, motor_R4_dir, timestep)
    motor_sim_R5 = MotorSim(motor_R5, sensor_R5, motor_R5_dir, timestep)
    motor_sim_R6 = MotorSim(motor_R6, sensor_R6, motor_R6_dir, timestep)

    motors_L = [motor_sim_L1, motor_sim_L2, motor_sim_L3, motor_sim_L4, motor_sim_L5, motor_sim_L6]
    motors_R = [motor_sim_R1, motor_sim_R2, motor_sim_R3, motor_sim_R4, motor_sim_R5, motor_sim_R6]

    return motors_L, motors_R

def arm_motor_define():
    motor_L1 = robot.getDevice('arm-pitch-l')
    motor_L2 = robot.getDevice('arm-roll-l')
    motor_L3 = robot.getDevice('elbow-pitch-l')

    sensor_L1 = robot.getDevice('arm-pitch-l_sensor')
    sensor_L2 = robot.getDevice('arm-roll-l_sensor')
    sensor_L3 = robot.getDevice('elbow-pitch-l_sensor')

    motor_L1_dir = 1.0
    motor_L2_dir = 1.0
    motor_L3_dir = 1.0

    motor_R1 = robot.getDevice('arm-pitch-r')
    motor_R2 = robot.getDevice('arm-roll-r')
    motor_R3 = robot.getDevice('elbow-pitch-r')

    sensor_R1 = robot.getDevice('arm-pitch-r_sensor')
    sensor_R2 = robot.getDevice('arm-roll-r_sensor')
    sensor_R3 = robot.getDevice('elbow-pitch-r_sensor')

    motor_R1_dir = 1.0
    motor_R2_dir = 1.0
    motor_R3_dir = 1.0

    motor_sim_L1 = MotorSim(motor_L1, sensor_L1, motor_L1_dir, timestep)
    motor_sim_L2 = MotorSim(motor_L2, sensor_L2, motor_L2_dir, timestep)
    motor_sim_L3 = MotorSim(motor_L3, sensor_L3, motor_L3_dir, timestep)

    motor_sim_R1 = MotorSim(motor_R1, sensor_R1, motor_R1_dir, timestep)
    motor_sim_R2 = MotorSim(motor_R2, sensor_R2, motor_R2_dir, timestep)
    motor_sim_R3 = MotorSim(motor_R3, sensor_R3, motor_R3_dir, timestep)

    motors_L = [motor_sim_L1, motor_sim_L2, motor_sim_L3]
    motors_R = [motor_sim_R1, motor_sim_R2, motor_sim_R3]

    return motors_L, motors_R

leg_motor_L, leg_motor_R = leg_motor_define()
arm_motor_L, arm_motor_R = arm_motor_define()

dt = 0;

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:

    # Process sensor data here.

    # Enter here functions to send actuator commands
    for i in range(6):
        leg_motor_L[i].set_position(0)
        leg_motor_R[i].set_position(0)

    for i in range(3):
        arm_motor_L[i].set_position(0)
        arm_motor_R[i].set_position(0)

    dt += 0.01
    # arm_motor_L[0].set_position(sin(dt) * 0.5)
    arm_motor_L[1].set_position(sin(dt) * 0.5)
    arm_motor_L[2].set_position(sin(dt) * 0.5)

    pass

# Enter here exit cleanup code.
