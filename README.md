# particle_filter_project

Ana Rath, William Zeng

## Writeup
First, we will start by initializing the particle cloud to consist of particles at random positions. We will draw from a random distribution to consist of our initial X_t. The test itself is the drawing from a random distribution. 

Next, we will update the position of the particles according to the changing location of the robot. For example, if the robot moves ahead by 1 unit, or turns clockwise by 90 degrees, we update the [x, y, Θ] locations. To test it, we could do something similar to the class exercise where we move the robot ahead by a fixed amount (say 1 unit forward), and see if the output of 'update_particles_with_motion_model' matches our expectation of the new particle location if we calculate it by hand. 

To calculate the importance weights, we use the Monte Carlo localization formula from class. To test the output of this function, we could do a similar exercise as class where we compute the expected output by hand and see if the function output matches it.
To normalize the important weights such that their sum adds up to 1, we can divide each weight by the sum of all the weights. This won’t require any testing. We can then resample particles from a normal distribution, proportional to the importance weights. 

We then get the estimated robot pose by updating our robot’s location by resampling more particles from the vicinity of particles that had higher importance weights. To test the 'update_estimated_robot_pose' function, we can get the 2D pose estimate from Ruiz and compare it with the output of our function. 

The resampling of particles from a normal distribution incorporates noise into our localization. To test this, we can see whether the resampled particle in each iteration differs from the particle location in the previous iteration. If any particles are the same, then noise has not been incorporated in the resampling. 


## Timeline:
Project Due Date: April 26 

Week 1  (till 04/20) : Record map, Initialize particle cloud, update position of the robots, Calculate importance weights - conduct required tests 

Week 2 (till due date): Resample particles, repeat iterations, update pose, optimize parameters 

## Objectives description (2-3 sentences): Describe the goal of this project.
  * The goal of this project is to enable the robot to locate itself in a given environment. The algorithm estimates the position and orientation of the robot given the environment. 
## High-level description (1 paragraph): At a high-level, describe how you solved the problem of robot localization. What are the main components of your approach?
  * The main components included:
    * Initializing a particle cloud to consist of a series of particles with varying positions and poses 
    * Next, we assign weights to the particles depending on the closeness of the particle locations with the robot’s sensor readings 
  * We also made sure to normalize these weights
  * After this, we updated the estimated robot pose by taking the sample average of the positions and orientations of our particles. 
  * We then resample the particles by drawing from a distribution with weighted probabilities. 
  * We then move the particles by the same amount the robot moves by
  * And repeat the process from Step #3. 
## For each of the main steps of the particle filter localization, please provide the following:

### Initialization of particle cloud:
 * Location: initialize_particle_cloud
 * Code description: For each particle in our total number of particles, we assign a random position and a random orientation. We then append this particle to the cloud. 
### Normalize particles:
 * Location: normalize_particles
 * Description: In order to make the weights add up to 1, we divide each weight by the total weight 
### Movement model:
 * Location: update_particles_with_motion_model:
 * Description: Here, we used the likelihood field model
   * This finds how much the 
### Measurement model:
 * Location: update_particle_weights_with_measurement_model
 * Description: Here we updated the weights using the likelihood field model.
### Resampling:
 * Location: Resample_particles and draw_random_sample helper function
 * Description: In this, we drew a random sample from our weighted list of the particle positions and their weights 
### Incorporation of noise:
 * Location: In the update_particles_with_motion_model function
 * Description: we included noise by estimating the alpha values proportionally to the robot’s position 
### Updating estimated robot pose:
 * Location: update_estimated_robot_pose 
 * Description: For each particle, we add up the x and y positions in order to find the sample average
  * We use the average x and y to create the average yaw using tan inverse.
  * We then return the average position consisting of the average x and y and the average orientation consisting of the quaternion from the new yaw

### Optimization of parameters:
Tweaking the alpha values in the incorporation of noise was done to optimize the parameters
We also tweaked the number of particles to optimize our parameters 

## Challenges (1 paragraph): Describe the challenges you faced and how you overcame them.
One challenge we faced was how each particle interacted with the robot position. In order to overcome this, we realize that our understaning of the pose object consisting of position and orientation, quaternions, and motion model was lacking. Therefore, we read more into documentations found in the linked textbook and consulted TAs in order to solve this. In addition, our odometry values seemed to deviate from what is printed from scan. We solved this by looking into where our values were changing in order to correct our math in those areas.

## Future work (1 paragraph): If you had more time, how would you improve your particle filter localization?
We would improve it by optimizing the parameters more. We feel like a lot of the parameters and variables such as alpha values were guess and check. Furthermore, we could've understood the map component of the project better to get the resolution of the environemnt. In addition, we could have implemented our motion model better. Our particles don't seem to always follow our robots orientation. I feel like we can also try to make our particles converge better. In addition, our particle only moves if our robot moves a certain distance. However, this distance is a big long for the small map we had, making the particles' movements less fluent. Therefore, I feel like we can change this bit of code to update the particles more frequently.

## Takeaways (at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.
  * Carefully study the parameters to check what information we have.
    * While implementing the functions, although we had the idea down I felt that we misused/misunderstood some variables such as understanding the individual position and orientation parameters of pose.
  * Do more research into quaternion
    * A lot of our questions to slack were how we would implement orientation change which had largely to do with quaternion and I felt like we did not have an understanding of that until later on in the project.

## GIF of our most successful attempt
