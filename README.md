## Robot Localization

### Implementation

For the 2nd mini-project for Introduction to Computational Robotics, we designed a particle filter to solve a robot locailzation challenge. Given a map, our goal was to locate the robot using lidar, odometery, and preexisting map data. Our final implementation of the particle filter shows clear convergence and with further tuning of variance and particle distribution we belief the partilce filter could be improved.

![Converge](documentation/convergence.gif)

### Design Decisons
##### Flexible Particle and Laser Samples
Our particle filter has some adjustable parameters, like the number of particles and the number of laser scan points used. We decided it would be useful to be able to adjust these parameters for debugging purposes, and it's also nice to have some flexibility when using the filter in general.


After we measure the distance between each point in the projected laser scan and the nearest object on the map, we use a bell curve function to convert that value to a weight.

![Flow](https://github.com/amfry/robot_localization/blob/master/documentation/flow.png)

![Part](https://github.com/amfry/robot_localization/blob/master/documentation/part.png)

### Challenges
##### Too Much Convergence ?
After distributing and resampling particles, the particle displayed in Rviz seemed to be get downsampled without us implementing that feature.  After confirming that our particle cloud was not changing in size while the filter was being run, we were stumped at what the problem could be. Ultimatley, at that point in our implemntation the particles were aggressivley converging and they were on top of one another.  By introducing random variance to particles during the resample phase, we were able to create a partilce cloud, then a few locations were all teh particles were on top of one another. We had not considered what successfully convergence might look like so we didn't indentify "downsampling" as anything other than a bug in the code.

##### Bag Files
In mini-project 1, we had used bag files as a recording tool and were intailly confused how the bag file could be the basis of the particle filter simulation.  It took us a while to get into a solid workflow and understand the difference between the robot model on the map and traveling red arrow.  It was also intially disorienting to consider how things like the laser_scan topic could be subscribed to without a neato being simulated in Gazebo. This confusion delayed how quickly we were able to start iterating on code.

### Future Improvements
Our filter could be improved by working to tightening the cloud of particles around the proposed robot position. This could be done by increase the weights of particles that have scan values that align with the robots.  Using normal distribution curves with a steeper cutoff would force more convergence.  To help tighten the cloud, we could also decrease the amount of varriance introduced during the resample step so the convergence is more apparent in the mean pose and in visualization.

To determine the coordinates and orientation of the predicted robot location, we are taking the mean of positions and angles for all of the particles within the particle cloud.  To increase the accuracy of the pose prediction, we could use standard deviation to select a value with a low standard deviation that is closer to the mean.

### Lessons Learned
1. This was our first time using a bag file as a starting point for designing an algorithm, and not just as a recording tool, so we had a learning curve at the beginning of the project to understand where the items were coming from that we were seeing in Rviz. By asking questions, we were able to clarify that the large red arrow in Rviz is the robot’s actual position as it was recorded, and the map was also from the bag file.

2. We quickly discovered that visualization is incredibly helpful, especially in Rviz. Our instinct is usually to print out notable values while debugging code, but this is less useful when working with hundreds of particle objects. We created markers that scale according to the weight of each particle and used these to better understand the behavior of our algorithm during the development stage. This occasionally led to some uncomfortable to look at visualizations.
![Weights](https://github.com/amfry/robot_localization/blob/master/documentation/weights.png)
