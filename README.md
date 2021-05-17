# Lab-Assignment-2---UR3-Shooting-Hoops
Using the UR3, the snake like robot will be able to track and shoot a ball to your target. 

The Black_Mamba Project uses a UR3 robot to simulate a targeting and shooting task in a basketball themed environment. 
In the Assignment_2_Main.m file, running it alongside our GUI will need some initial inputs to set the stage.

Setting the hoops initial position along the slider with incriments of 0.1 t0 -0.1. This is due to the codes limiting velocity output and tracking ability. If the hoop is set too far from the robot. An error will state that:

a. A velocity was not found when calculating trajectory.
b. The found velocity to too low or too high for a cartisian path to be set for ther UR3

Another initialising prompt will allow the setting to contain obsticals and possible collisons. If yes, another prompt may appear and dictate whether you want to proceed or abort the sequence if you want to 'manually' remove the objects. 

The Collision detection is fundermental and will not respond to randomly placed objects. We've assigned invisable hitboxes to simulated objects to give the perception of detection and avoidance.
The sensory equiptment on the UR3 is also rudementry and does not detect object based point rather than an invisable point that slides along with the hoop.
THe equipement we needed to program this effectivly would use an RGB-D sensor, which is able to tackle all listed problems above. 

Overall, the robot sensors, positions, calculates trajectory and is able to move at the appropriate velocities to throw the ball towards the target. 

