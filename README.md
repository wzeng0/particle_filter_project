# particle_filter_project

Ana Rath, William Zeng

## Writeup
First, we will start by initializing the particle cloud to consist of particles at random positions. We will draw from a random distribution to consist of our initial X_t. The test itself is the drawing from a random distribution. 

Next, we will update the position of the particles according to the changing location of the robot. For example, if the robot moves ahead by 1 unit, or turns clockwise by 90 degrees, we update the [x, y, Θ] locations. To test it, we could do something similar to the class exercise where we move the robot ahead by a fixed amount (say 1 unit forward), and see if the output of '''update_particles_with_motion_model''' matches our expectation of the new particle location if we calculate it by hand. 

To calculate the importance weights, we use the Monte Carlo localization formula from class. To test the output of this function, we could do a similar exercise as class where we compute the expected output by hand and see if the function output matches it.
To normalize the important weights such that their sum adds up to 1, we can divide each weight by the sum of all the weights. This won’t require any testing. We can then resample particles from a normal distribution, proportional to the importance weights. 

We then get the estimated robot pose by updating our robot’s location by resampling more particles from the vicinity of particles that had higher importance weights. To test the '''update_estimated_robot_pose''' function, we can get the 2D pose estimate from Ruiz and compare it with the output of our function. 

The resampling of particles from a normal distribution incorporates noise into our localization. To test this, we can see whether the resampled particle in each iteration differs from the particle location in the previous iteration. If any particles are the same, then noise has not been incorporated in the resampling. 


## Timeline:
Project Due Date: April 26 

Week 1  (till 04/20) : Record map, Initialize particle cloud, update position of the robots, Calculate importance weights - conduct required tests 

Week 2 (till due date): Resample particles, repeat iterations, update pose, optimize parameters 
