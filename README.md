# LEGO 
The repository follows along with the paper "LEGO: Leveraging Experience in Roadmap
Generation for Sampling-Based Planning" by Rahul Kumar, Aditya Mandalika, Sanjiban Choudhury and Siddhartha S. Srinivasa of Personal Robotics Lab, University of Washington, submitted to IRoS 2019.

The IPython notebook learns samples along bottleneck and diverse regions, thus maximizing the likelihood of a feasible path while maintaining sub optimality. We train a CVAE model with the target samples conditioned on the problem's initial state, goal state, and an occupancy grid of the obstacles.
