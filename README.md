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
After distributing and resampling particles, the particle displayed in Rviz seemed to be get downsampled without us implementing that feature.  After confirming that all 

### Future Improvements

### Lessons Learned
1. This was our first time using a bag file as a starting point for designing an algorithm, and not just as a recording tool, so we had a learning curve at the beginning of the project to understand where the items were coming from that we were seeing in Rviz. By asking questions, we were able to clarify that the large red arrow in Rviz is the robot’s actual position as it was recorded, and the map was also from the bag file.

2. We quickly discovered that visualization is incredibly helpful, especially in Rviz. Our instinct is usually to print out notable values while debugging code, but this is less useful when working with hundreds of particle objects. We created markers that scale according to the weight of each particle and used these to better understand the behavior of our algorithm during the development stage. This occasionally led to some uncomfortable to look at visualizations.
![Weights](https://github.com/amfry/robot_localization/blob/master/documentation/weights.png)
