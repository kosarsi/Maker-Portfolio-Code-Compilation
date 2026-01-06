# Maker-Portfolio-Code-Compilation

For all seasons, the state machine is visible in the entire RUN program (for decode, it's RUNRed because 
of different tags for the different alliance goals necessitating multiple run programs)

Decode:
Sensor fusion of odometry and camera is on line 127 of HardwareControl.java
Gain scheduling: the gains start on line 15 of RUNRed.java, and are used starting on line 121 and line 167 
Regression Model is on line 249 of HardwareControl.java
Laser sensor rumbling is on line 88 of RUNRed.java
The simple autonomous structure, enabled by NextFTC, is in TwelveBallRed.java. All the subsystem code to control
actuators was written by me, NextFTC just schedules the commands as I program it to do so

Into The Deep:
Homography perspective transform is on line 29 of YellowDetect.py - for context, it was done empirically by 
setting a rectangular sheet on the ground and mapping the corners of the sheet in the image frame to the 
corners of the screen, accounting for any camera warps or imperfections 
Motion profile starts on line 226 of HardwareControl.java
Full State Feedback starts on line 361 of RUN.java
Hall Effect Sensor programming is on line 70 of HardwareControl.java and line 340 of RUN.java
Full seven sample autonomous is provided, using the NextFTC command base just like Decode

Centerstage:
Pose update can be found on lines 233, 352, and 470 of RedLeftCycle.java
Beam breaks can be found starting on line 458 of RUN.java
Backdrop avoidance rumbling is found on line 128 of RUN.java

Powerplay:
Autoglide is found on line 374
Claw limit switch is found on line 478
AprilTag detection is on line 151 of RightSubstation.java
