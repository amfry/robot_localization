## Robot Localization

### Project Goal

The goal of this project was to develop a particle filter algorithm for the purpose of robot localization.

### Implementation

![](documentation/convergence.gif)


### Design Decisons

Our particle filter has some adjustable parameters, like the number of particles and the number of laser scan points used. We decided it would be useful to be able to adjust these parameters for debugging purposes, and it's also nice to have some flexibility when using the filter in general.

After we measure the distance between each point in the projected laser scan and the nearest object on the map, we use a bell curve function to convert that value to a weight.

### Challenges

### Improvements

### Lessons Learned
1)
2) 
3) Visualize!
We used visualization with particle arrows and markers at multiple points to get a better understanding of how the scripts were working.  For debugging our resampe function, we published Markes proprtional to the weght of the asscoiated particle. This led to some unhelpful and uncomfortable to look at visualizations.
![Weights](https://github.com/amfry/robot_localization/blob/master/documentation/weights.png)
