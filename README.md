# Autonomous Nano Drone
------------
An open-source, off-the-shelf drone with easily replaceable parts controlled autonomously indoors using a Raspberry Pi Zero as the on-board computer.

The point of this project is to make a drone whose parts can be sent individually to someone who wants to make a drone, with any of those parts being replaceable and the drone still working.

We're using two parallel approaches: stabilization using Optical Flow sensors, and WhyCon.

Optical flow sensors are not very good at stabilizing drones, but through WhyCon we can achieve millimeter precision. Hence, we aim to use both methods and 
do onboard sensor fusion, for more accurate values.
