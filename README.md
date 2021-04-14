# Particle Filter Localization Project 

## Project Plan
Stephanie Kim ([stephaniekim@uchicago.edu](mailto:stephaniekim@uchicago.edu))

Zhou Xing ([zhouxing@uchicago.edu](mailto:zhouxing@uchicago.edu))

### **`initialize_particle_cloud()`**

We will create and visualize a random uniform distribution of particles on the "map", likely using a ROS and/or Python package to do so.

### **`update_particles_with_motion_model()`**

We will track the distance and direction the robot travels, represented by (x, y, θ). With this starting and ending coordinates of the robot, the total distance traveled can be calculated. All of the particles will then move this distance in the same direction θ as the robot.

### **`update_particle_weights_with_measurement_model()`**

From the '/scan' topic, we will gather either the minimum or average data.ranges measurement across a 10° angle range at 0°, 90°, 180°, and 270° (e.g. the angle ranges for which we measure data.ranges would be 355-5°, 85-95°, etc.), which corresponds to the distance of the closest object in front of, to the left & right of, and behind the robot. These would be the robot's "actual" sensor readings. We would also capture each particle's "sensor readings". Using the measurement model below, we would then calculate the importance weights for each particle.

$$w_t^{[m]} = \frac{1}{\sum_{i = 0}^2 \textrm{abs}(z_t[i] - z_t^{[m]}[i])}$$

### **`normalize_particles()`**

We will normalize the weight of each particle by dividing them with the sum of the weights, i.e. $w^*_i = \frac{w_i}{\sum{w_i}}$, so that $\sum_i w_i^* = 1$. 

### **`resample_particles()`**

With the normalized weight, we take the $\{(particle_i, w_i^*)\}$ as a new discrete distribution, and resample from it so that the new collection has the same number of particles as the initial particle cloud. We probably will plot the resampled collections on the map, and expect a less spread out distribution compared to the previous one.

### **`update_estimated_robot_pose()`**

We need to determine an exact new state for the robot from the resampled particles. We refer to [this tutorial]([https://jakevdp.github.io/PythonDataScienceHandbook/05.13-kernel-density-estimation.html](https://jakevdp.github.io/PythonDataScienceHandbook/05.13-kernel-density-estimation.html)) about kernel density to  "convert the discrete $\{(particle_i, w_i^*)\}$ distribution into a continuous one", and pick the location with the highest probability density to be the result.

### Incorporating N**oise**

We will sample noise component from a plain gaussian distribution for each particle. We would prioritize adding these noises into the motion model, seeing how that goes, and then potentially incorporate noise into the measurement model if it makes sense.

### **Timeline**

**`initialize_particle_cloud()`: April 18**

**`update_particles_with_motion_model()`:** **April 19**

**`update_particle_weights_with_measurement_model()`: April 21**

**`normalize_particles()`: April 22**

**`resample_particles()`: April 23**

**`update_estimated_robot_pose()`: April 24**

Due Apr 26, 11:00am CST