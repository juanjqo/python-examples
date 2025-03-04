#!/bin/python3
"""
(C) Copyright 2025 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)

Instructions:

Open the DQ_Robotics_lab.ttt scene in CoppeliaSim before running this script.
"""
from dqrobotics import*
from dqrobotics.interfaces.coppeliasim import DQ_CoppeliaSimInterfaceZMQ
from math import sin, cos, pi, pow
import numpy as np
import time

def main() -> None:
    cs = DQ_CoppeliaSimInterfaceZMQ()
    try:
        cs.connect("localhost", 23000, 1000) # "127.0.0.1", 19997, 1000, 10
        cs.set_stepping_mode(True)
        time.sleep(0.1)
        cs.start_simulation()

        # Define some parameters to compute varying-time trajectories
        a = 1
        freq = 0.1
        time_simulation_step = 0.05
        for i in range(300):
            t = i * time_simulation_step
            # Define a varying-time position based on the Lemniscate of Bernoulli
            p = (a * cos(t) / (1 + pow(sin(t),2))) * i_ + (a * sin(t) * cos(t) / (1 + pow(sin(t),2))) * j_

            # Define a varying-time orientation
            r = cos(2 * pi * freq * t) + k_ * sin(2 * pi * freq * t)

            # Built a varying-time unit dual quaternion
            x = r + 0.5 * E_ * p * r

            # Set the object pose in CoppeliaSim
            cs.set_object_pose("/coffee_drone", x)

            # Read the pose of an object in CoppeliaSim to set the pose of
            # another object with a constant offset.
            xread = cs.get_object_pose("/coffee_drone")
            xoffset = 1 + 0.5 * E_ * (0.5 * i_)
            xnew = xread * xoffset
            cs.set_object_pose("/Frame_x", xnew)

            # Set the target position of the first joint of the UMIRobot arm
            target_position = [sin(2 * pi * freq * t)]
            cs.set_joint_target_positions(["UMIRobot/UMIRobot_joint_1"],  target_position)

            # Set the target position of the third joint of the UR5 robot
            cs.set_joint_target_positions(['UR5/link/joint/link/joint'], target_position)

            # Set the target velocity of the first joint of the Franka Emika Panda
            target_velocity = [2 * pi * freq * cos(2 * pi * freq * t)]
            cs.set_joint_target_velocities(['Franka/joint'], target_velocity)

            # Set the torque of the fifth joint of the Franka Emika Panda
            torque = [sin(2 * pi * freq * t)]
            cs.set_joint_torques(['Revolute_joint'], torque)

            # Set the target velocities of the Pioneer wheels
            cs.set_joint_target_velocities(['PioneerP3DX/rightMotor'], [0.1])
            cs.set_joint_target_velocities(['PioneerP3DX/leftMotor'], [0.2])

            # Trigger a simulation step in CoppeliaSim
            cs.trigger_next_simulation_step()

        cs.stop_simulation()

    except (Exception, KeyboardInterrupt) as e:
        print(e)
        cs.stop_simulation()
        pass

if __name__ == "__main__":
    main()

