## Robot Localization

### Project Goal

### Implementation

### Design Decisons

### Challenges

### Improvements

### Lessons Learned

##### Implmentation Plan

We are planning on using pf_scaffolded.py for the project.

1) Neato is a location

2) Evenly distibute particle across the map

3) Using neato scan data, assign a weight to each particle

4) Using the weigts, resample particles so that particles are concentrating in highlt weighted portions of the map

5) Neato moved to a new location

Step 3-5 repeats to track the estimated locilization of the neato.

