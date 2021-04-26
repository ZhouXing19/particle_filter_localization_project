# Particle Filter Localization Project 

## Project Plan
Stephanie Kim ([stephaniekim@uchicago.edu](mailto:stephaniekim@uchicago.edu))

Zhou Xing ([zhouxing@uchicago.edu](mailto:zhouxing@uchicago.edu))

### **`initialize_particle_cloud()`**

We will create and visualize a random uniform distribution of particles on the "map", likely using a ROS and/or Python package to do so.
- Testing: We would visualize the particle cloud on the map, and try to make sure it looks uniformly distributed in the map.

### **`update_particles_with_motion_model()`**

We will track the distance and direction the robot travels, represented by (x, y, θ). With this starting and ending coordinates of the robot, the total distance traveled can be calculated. All of the particles will then move this distance in the same direction θ as the robot.

  - Testing: We would plot the particles before and after the motion, make sure all the particles are moved to the same direction with the same step size.

### **`update_particle_weights_with_measurement_model()`**

From the '/scan' topic, we will gather either the minimum or average data.ranges measurement across a 10° angle range at 0°, 90°, 180°, and 270° (e.g. the angle ranges for which we measure data.ranges would be 355-5°, 85-95°, etc.), which corresponds to the distance of the closest object in front of, to the left & right of, and behind the robot. These would be the robot's "actual" sensor readings. We would also capture each particle's "sensor readings". Using the measurement model below, we would then calculate the importance weights for each particle.

$$w_t^{[m]} = \frac{1}{\sum_{i = 0}^2 \textrm{abs}(z_t[i] - z_t^{[m]}[i])}$$

- Testing: We would randomly pick some particles and calculate the weights by hand, and compare our results with the codes' output, to make sure the calculation is correct.

### **`normalize_particles()`**

We will normalize the weight of each particle by dividing them with the sum of the weights, i.e. $w^*_i = \frac{w_i}{\sum{w_i}}$, so that $\sum_i w_i^* = 1$. 

- Testing: Similar to the previous part, we will pick the element and test if the sum of weights is 1.

### **`resample_particles()`**

With the normalized weight, we take the $\{(particle_i, w_i^*)\}$ as a new discrete distribution, and resample from it so that the new collection has the same number of particles as the initial particle cloud. 

- Testing: We probably will plot the resampled collections on the map, and expect a less spread out distribution compared to the previous one.



### **`update_estimated_robot_pose()`**

We need to determine an exact new state for the robot from the resampled particles. We refer to [this tutorial]([https://jakevdp.github.io/PythonDataScienceHandbook/05.13-kernel-density-estimation.html](https://jakevdp.github.io/PythonDataScienceHandbook/05.13-kernel-density-estimation.html)) about kernel density to  "convert the discrete $\{(particle_i, w_i^*)\}$ distribution into a continuous one", and pick the location with the highest probability density to be the result.

- Testing: Not sure about the testing for this part, but probably we will plot the densest particles from resampling, and check by eye. 

### Incorporating **Noise**

We will sample noise component from a plain gaussian distribution for each particle. We would prioritize adding these noises into the motion model, seeing how that goes, and then potentially incorporate noise into the measurement model if it makes sense.

-Testing: Compare the particles before and after noise implementation.

### **Timeline**

**`initialize_particle_cloud()`: April 18**

**`update_particles_with_motion_model()`:** **April 19**

**`update_particle_weights_with_measurement_model()`: April 21**

**`normalize_particles()`: April 22**

**`resample_particles()`: April 23**

**`update_estimated_robot_pose()`: April 24**

Due Apr 26, 11:00am CST

## Objectives
The goal of our project is to use particle filter localization in order to estimate the location of our robot within the map as accurately as possible. To increase accuracy, we want to be able to account for noise in the environment, as well.

## High-level description
Through a likelihood field for range finders algorithm, we gradually localize the particles in the map into one cloud that should track the robot's actual location in the map.

## Steps of Particle Filter Localization

### Initialization of particle cloud

**Code location:** Lines 188-241, right after the `abs_loc_2_map_loc()` function and before `normalize_particles()`.

**Code description:** This step is handled by `initialize_particle_cloud()`. Using methods from the LikelihoodField class, we establish a bounding box on the map according to the location of obstacles. For each of 1000 particles (value of `num_particles`), we then initialize the particle's pose to a random x and y value within the bounding box. We check if this pose is outside of the bounds. If it is, we reinitialize its pose until it acquires an x and y value within the bounding box. We set the yaw of the particle equal to a random heading between 0 and 360 degrees, converting the (x, y, theta) of the particle into a quaternion. We create a new instance of the Particle class using this particle's pose and orientation, assigning it an arbitrary weight of 1 to begin with. This instance is appended to `particle_cloud`. Once all of the particles have been initialized, the weights of the particles in the particle cloud are then normalized using `normalize_particles`, which takes each particle and divides its weight by the sum of all particle weights. The particle cloud is published.

### Movement model

**Code location:** Lines 402-446, the last function in the ParticleFilter class.

**Code description:** This step is handled by `update_particles_with_motion_model()`. From the robot's odometry, we pass in its change in x-position (`dx`), change in y-position (`dy`), and change in z-orientation (`dyaw`) as arguments to our function. For each particle in our particle cloud, we simply change its orientation by adding `dyaw` to the particle's current yaw. The position change of the particle is a little more complicated, where we use trigonometry to make sure the particle moves in the direction of its heading, not in the exact same direction as the robot. With the new x-position, y-position, and yaw for the particle, we create a new Particle instance with these values for its pose attribute and assign it to the current particle.

### Measurement model

**Code location:** Lines 369-396, right after `update_estimated_robot_pose()`.

**Code description:** This step is handled by `update_particle_weights_with_measurement_model()`. Essentially, we used the likelihood field for range finders algorithm from class in order to update each particle's importance weight.

### Resampling

**Code location:** Lines 278-283, right before `robot_scan_received()`.

**Code description:** This step is handled by `resample_particles()`. Our function uses `draw_random_sample()` to draw a resampling of particles from our particle cloud based on particle weights. `draw_random_sample()` uses `np.random.choice()` to choose a random array of `len(self.particle_cloud)` particles using the particle weights as probabilities passed as an argument. The returned array from `draw_random_sample()` is copied using `deepcopy` into the original particle cloud. 

### Incorporation of noise

**Code location:** Lines 28-39 for noise generation function `noise_in_range()`, and Lines 424-426 (inside `update_particles_with_motion_model()`) for actual incorporation of noise.

**Code description:** The generation of Gaussian noise was handled by `noise_in_range()`.`noise_in_range()` uses the `numpy.random` module's `normal` function draws a random sample from a Gaussian distribution. This sample is returned as a noise value that is later utilized in updating each particle's motion. Lines 424-426 are where the Gaussian noise is actually incorporated; noise is generated for the change in x-position, change in y-position, and change in yaw, and added to the new x-position, y-position, and yaw for the current particle.

### Estimating robot pose

**Code location:** Lines 358-365, right after `robot_scan_received()`.

**Code description:** This step is handled by `update_estimated_robot_pose()`. The function simply takes the particle of maximum importance weight from our particle cloud and sets the robot's estimated pose to this particle's pose.

### Optimization of parameters

**Code location:** Lines 111, 121, 341, 373 (the most obvious optimizations)

**Code description:** At the above lines, we modified some of the key parameters of the particle filter to assist in a smoother and more accurate localization of the particles. On lines 111 and 121, we changed the number of particles and linear movement threshold, respectively, to be much lower than their initial values. Doing this allowed the particles to move with the robot at lower speeds and helped the computer process the amount of moving particles. (The initial value, 10000, kept giving us runtime errors.) On line 341, we modified the parameters of `update_particles_with_motion_model()` so that we didn't have to recalculate the robot's change in movement everytime we wanted to move the particle cloud. Instead, we just passed in the robot's changed position and orientation. On line 373, we only used 4 cardinal directions. Again, this helped the computer process the movement of the particles without being overloaded with information, which it was often experiencing in the earlier stages of the project.

## Challenges
One of our challenges at the start was getting the particles to initialize within the bounds of the house; originally, we had particles spawning at any location on the map, including atop obstacles and outside the house. We had to refer to some online ROS resources in order to figure out the details of `OccupancyGrid` and how we can use the values returned from the grid's data to determine whether or not a particle is spawning in an invalid area. Anytime a particle was deemed to have spawned in an invalid area, we simply randomized its position again until it no longer spawned in an invalid area.
Another large challenge was a bug that Zhou experienced where the particles wouldn't concentrate into smaller clouds, but simply disappear from the map. This wasn't an issue that was generated by a specific bit of code; it really only solved itself through constant tweaking of the code to make the particle motion smoother and less susceptible to noise. We also had to dramatically decrease the number of particles we were working with from 10,000 to 1000, since we both kept getting an "extrapolation into the past" error from ROS.
Unrelated to the actual coding, Stephanie faced a last-minute breakdown of RViz, where RViz wasn't displaying any movement of the particles despite the code being identical to Zhou's and working on Zhou's machine. This made Stephanie very crestfallen, as she could not help with the testing and fine-tuning for most of Sunday, so she could really only draft code and hope it worked on Zhou's machine.

## Future work
For the future, we want to try out the navigation tools and seeing if the robot successfully moves out of the house. This was an optional exercise that we didn't get to, so it would be interesting to see if it would run successfully based on our current code. We also would probably continue to tweak the code and the noise incorporation to see if we could get a tighter particle cloud around the robot. We'd also continue to optimize our particle cloud localization further, particularly the movement of particles that don't converge to the final central cloud near the robot. In some runs, particles will move across objects or out of bounds, which we tried to mitigate in our code the best we could given the time constraint. Ideally, we want all of the particles to behave properly on nearly every run.

## Takeaways
1. When it comes to pair programming, consistent and specific communication is definitely key to getting the project done. We would often Slack each other when we were working on the project and when we anticipated we would not be able to work on the project, as well as everytime either one of us pushed to the Git repo or made significant code changes. One thing we could've done more is possibly coordinating who was working on what part of the code to maximize efficiency. Sometimes, we would work on the same function at the same time (and end up with nearly identical code!), so it would've been more useful in those situations to coordinate, say, one person working on the function (driving) and another checking it for accuracy (navigating).
2. It really helps to set a timeline for projects! Though we didn't follow it to a T, having a general idea of when we at least wanted each function written helped keep us on track. It also ensured that we weren't scrambling to write any particular function at the last minute, since the skeleton of the code was already there. This is certainly something to utilize for future individual projects, as well.
3. A good partner project involves a mix of dividing and conquering as well as collaboration on specific aspects, too. If it's all divide-and-conquer, certain partners won't get experience with certain aspects of the project, and it makes it much harder when one partner gets stuck and the other isn't caught up on their work. On the other hand, it's also difficult to work on a project synchronously the whole time, so some asynchronous division of work is very necessary for efficiency. 