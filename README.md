# IKRC-for-Kerbal
Inverse Kinematics Robot Controller for Kerbal

The IKRC uses the InfernalRobotics API. I needed to modify a bit for the correct operation (InfernalRoboticsMod).
Ca2LeeCam is a camera plugin for Latch End Effector.
The Canadarm2 directory contains the Kerbal Parts and the built-in robot arm.
The IkRobotController directory contains the source of the plugin in C#.

I'm not too strong on github desktop. That is why the summary is the same.

The WireSnareLock plugin can disable TRF_CA2_LEE_wCam dockingNode and activate the wire-snare device to capture the PDGF. WireSnareLock support a physical connection with the vessel, but don't make logic connection.

I changed the IKRC control display.

The IKRC plugin expanded with Virtual EndEffector function (VEE).
